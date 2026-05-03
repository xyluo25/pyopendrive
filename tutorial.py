"""Smoke-test the pure-Python OpenDRIVE port with the repository map.

Run from the repository root:

    python -m pyopendrive.example_chatt
"""

import pyopendrive as odr


if __name__ == "__main__":
    xodr = "./datasets/chatt.xodr"
    odr_map = odr.readXodr(xodr)

    roads = odr_map.getRoads()
    junctions = odr_map.getJunctions()
    graph = odr_map.getRoutingGraph()

    lane_count = len(odr_map.getLanes())
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
