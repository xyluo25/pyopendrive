# ##############################################################
# Created Date: Saturday, May 2nd 2026
# Contact Info: luoxiangyong01@gmail.com
# Author/Copyright: Mr. Xiangyong Luo
# ##############################################################

"""Tutorial: create a small OpenDRIVE map from an empty OpenDriveMap.

This example builds the same map in two forms:

1. Python objects, so you can query roads, lanes, geometry, junctions,
   objects, signals, meshes, and routing immediately.
2. OpenDRIVE XML nodes, so ``save_xodr`` can write a real ``.xodr`` file.

The reader normally derives lane borders while parsing XML. When building
manually, we set those simple borders ourselves.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import pyopendrive as odr


xodr = odr.xodr


def add_constant_spline(spline: xodr.CubicSpline, s0: float, value: float) -> None:
    spline.s0_to_poly[s0] = xodr.Poly3.from_odr(s0, value, 0.0, 0.0, 0.0)


def create_simple_lane_section(
    road: xodr.Road,
    *,
    lane_width: float = 3.5,
) -> xodr.LaneSection:
    """Add one center lane, one left lane, and one right lane to a road."""

    section = xodr.LaneSection(road.id, 0.0)

    center_lane = xodr.Lane(road.id, section.s0, 0, type="none")
    left_lane = xodr.Lane(road.id, section.s0, 1, type="driving")
    right_lane = xodr.Lane(road.id, section.s0, -1, type="driving")

    add_constant_spline(center_lane.lane_width, section.s0, 0.0)
    add_constant_spline(center_lane.outer_border, section.s0, 0.0)

    add_constant_spline(left_lane.lane_width, section.s0, lane_width)
    add_constant_spline(left_lane.outer_border, section.s0, lane_width)
    left_lane.roadmark_groups.append(
        xodr.RoadMarkGroup(
            road.id,
            section.s0,
            left_lane.id,
            width=0.12,
            s_offset=0.0,
            type="solid",
            weight="standard",
            color="white",
            lane_change="none",
        )
    )

    add_constant_spline(right_lane.lane_width, section.s0, lane_width)
    add_constant_spline(right_lane.outer_border, section.s0, -lane_width)
    right_lane.roadmark_groups.append(
        xodr.RoadMarkGroup(
            road.id,
            section.s0,
            right_lane.id,
            width=0.12,
            s_offset=0.0,
            type="broken",
            weight="standard",
            color="white",
            lane_change="both",
        )
    )

    section.id_to_lane[center_lane.id] = center_lane
    section.id_to_lane[left_lane.id] = left_lane
    section.id_to_lane[right_lane.id] = right_lane
    road.s_to_lanesection[section.s0] = section

    return section


def create_straight_road(
    *,
    road_id: str,
    name: str,
    length: float,
    x: float,
    y: float,
    heading: float,
    junction: str = "-1",
) -> xodr.Road:
    """Create a straight road with plan-view geometry and basic lanes."""

    road = xodr.Road(road_id, length, junction, name)
    road.ref_line.s0_to_geometry[0.0] = xodr.Line(0.0, x, y, heading, length)
    add_constant_spline(road.ref_line.elevation_profile, 0.0, 0.0)
    add_constant_spline(road.lane_offset, 0.0, 0.0)
    add_constant_spline(road.superelevation, 0.0, 0.0)
    road.crossfall.s0_to_poly[0.0] = xodr.Poly3.from_odr(0.0, 0.0, 0.0, 0.0, 0.0)
    road.crossfall.sides[0.0] = "both"
    road.s_to_type[0.0] = "town"
    road.s_to_speed[0.0] = xodr.SpeedRecord(max="35", unit="mph")
    create_simple_lane_section(road)
    return road


def road_to_xml(road: xodr.Road) -> ET.Element:
    """Serialize the small subset used by this tutorial."""

    road_node = ET.Element(
        "road",
        {
            "name": road.name,
            "length": str(road.length),
            "id": road.id,
            "junction": road.junction,
            "rule": "RHT",
        },
    )

    link_node = ET.SubElement(road_node, "link")
    if road.predecessor.type != "none":
        ET.SubElement(
            link_node,
            "predecessor",
            {
                "elementType": road.predecessor.type,
                "elementId": road.predecessor.id,
                "contactPoint": road.predecessor.contact_point,
            },
        )
    if road.successor.type != "none":
        ET.SubElement(
            link_node,
            "successor",
            {
                "elementType": road.successor.type,
                "elementId": road.successor.id,
                "contactPoint": road.successor.contact_point,
            },
        )

    road_type_node = ET.SubElement(road_node, "type", {"s": "0", "type": "town"})
    speed = road.s_to_speed[0.0]
    ET.SubElement(road_type_node, "speed", {"max": speed.max, "unit": speed.unit})

    plan_view = ET.SubElement(road_node, "planView")
    for geom in road.ref_line.get_geometries():
        geom_node = ET.SubElement(
            plan_view,
            "geometry",
            {
                "s": str(geom.s0),
                "x": str(geom.x0),
                "y": str(geom.y0),
                "hdg": str(geom.hdg0),
                "length": str(geom.length),
            },
        )
        ET.SubElement(geom_node, "line")

    elevation_profile = ET.SubElement(road_node, "elevationProfile")
    ET.SubElement(
        elevation_profile,
        "elevation",
        {"s": "0", "a": "0", "b": "0", "c": "0", "d": "0"},
    )

    lanes_node = ET.SubElement(road_node, "lanes")
    ET.SubElement(
        lanes_node,
        "laneOffset",
        {"s": "0", "a": "0", "b": "0", "c": "0", "d": "0"},
    )
    section_node = ET.SubElement(lanes_node, "laneSection", {"s": "0"})
    for side_name, lane_ids in (("left", [1]), ("center", [0]), ("right", [-1])):
        side_node = ET.SubElement(section_node, side_name)
        section = road.get_lanesection(0.0)
        for lane_id in lane_ids:
            lane = section.get_lane(lane_id)
            lane_node = ET.SubElement(
                side_node,
                "lane",
                {"id": str(lane.id), "type": lane.type, "level": "false"},
            )
            if lane.id != 0:
                ET.SubElement(
                    lane_node,
                    "width",
                    {
                        "sOffset": "0",
                        "a": str(abs(lane.outer_border.get(0.0))),
                        "b": "0",
                        "c": "0",
                        "d": "0",
                    },
                )
            for group in lane.roadmark_groups:
                ET.SubElement(
                    lane_node,
                    "roadMark",
                    {
                        "sOffset": str(group.s_offset),
                        "type": group.type,
                        "weight": group.weight,
                        "color": group.color,
                        "laneChange": group.lane_change,
                    },
                )

    if road.id_to_object:
        objects_node = ET.SubElement(road_node, "objects")
        for obj in road.id_to_object.values():
            ET.SubElement(
                objects_node,
                "object",
                {
                    "id": obj.id,
                    "s": str(obj.s0),
                    "t": str(obj.t0),
                    "zOffset": str(obj.z0),
                    "length": str(obj.length),
                    "width": str(obj.width),
                    "height": str(obj.height),
                    "type": obj.type,
                    "name": obj.name,
                },
            )

    if road.id_to_signal:
        signals_node = ET.SubElement(road_node, "signals")
        for signal in road.id_to_signal.values():
            ET.SubElement(
                signals_node,
                "signal",
                {
                    "id": signal.id,
                    "name": signal.name,
                    "s": str(signal.s0),
                    "t": str(signal.t0),
                    "dynamic": str(signal.is_dynamic).lower(),
                    "zOffset": str(signal.zOffset),
                    "value": str(signal.value),
                    "height": str(signal.height),
                    "width": str(signal.width),
                    "orientation": signal.orientation,
                    "country": signal.country,
                    "type": signal.type,
                    "subtype": signal.subtype,
                    "unit": signal.unit,
                    "text": signal.text,
                },
            )

    return road_node


def junction_to_xml(junction: xodr.Junction) -> ET.Element:
    junction_node = ET.Element("junction", {"name": junction.name, "id": junction.id})
    for connection in junction.id_to_connection.values():
        connection_node = ET.SubElement(
            junction_node,
            "connection",
            {
                "id": connection.id,
                "incomingRoad": connection.incoming_road,
                "connectingRoad": connection.connecting_road,
                "contactPoint": connection.contact_point,
            },
        )
        for lane_link in connection.lane_links:
            ET.SubElement(
                connection_node,
                "laneLink",
                {"from": str(lane_link.from_lane), "to": str(lane_link.to_lane)},
            )
    for priority in junction.priorities:
        ET.SubElement(junction_node, "priority", {"high": priority.high, "low": priority.low})
    for controller in junction.id_to_controller.values():
        ET.SubElement(
            junction_node,
            "controller",
            {
                "id": controller.id,
                "type": controller.type,
                "sequence": str(controller.sequence),
            },
        )
    return junction_node


def build_demo_map() -> odr.OpenDriveMap:
    """Build a tiny map from scratch."""

    odr_map = odr.OpenDriveMap()
    ET.SubElement(
        odr_map.root,
        "header",
        {
            "revMajor": "1",
            "revMinor": "4",
            "name": "manual_tutorial",
            "version": "1.00",
            "date": "2026-05-02",
            "north": "10",
            "south": "-10",
            "east": "80",
            "west": "0",
        },
    )

    main_road = create_straight_road(
        road_id="1",
        name="main_road",
        length=50.0,
        x=0.0,
        y=0.0,
        heading=0.0,
    )
    main_road.successor = xodr.RoadLink("10", "junction", "start")
    main_road.id_to_object["stop_bar"] = xodr.RoadObject(
        main_road.id,
        "stop_bar",
        43.0,
        -1.75,
        0.0,
        0.4,
        0.4,
        3.5,
        0.0,
        0.02,
        0.0,
        0.0,
        0.0,
        type="roadMark",
        name="Stop bar",
    )
    main_road.id_to_signal["speed_35"] = xodr.RoadSignal(
        main_road.id,
        "speed_35",
        "Speed limit",
        10.0,
        -4.0,
        False,
        1.0,
        35.0,
        0.7,
        0.5,
        0.0,
        0.0,
        0.0,
        "+",
        "US",
        "274",
        "35",
        "mph",
        "35",
    )

    connector = create_straight_road(
        road_id="2",
        name="connector_road",
        length=25.0,
        x=50.0,
        y=0.0,
        heading=0.0,
        junction="10",
    )
    connector.predecessor = xodr.RoadLink("1", "road", "end")

    junction = xodr.Junction("simple_junction", "10")
    connection = xodr.JunctionConnection(
        "0",
        incoming_road=main_road.id,
        connecting_road=connector.id,
        contact_point="start",
    )
    connection.lane_links.append(xodr.JunctionLaneLink(from_lane=-1, to_lane=-1))
    junction.id_to_connection[connection.id] = connection
    junction.priorities.append(xodr.JunctionPriority(high=main_road.id, low=connector.id))
    junction.id_to_controller["ctrl_0"] = xodr.JunctionController(
        "ctrl_0",
        "signal",
        1,
    )

    main_road.xml_node = road_to_xml(main_road)
    connector.xml_node = road_to_xml(connector)
    odr_map.addRoad(main_road)
    odr_map.addRoad(connector)
    odr_map.addJunction(junction)

    return odr_map


def main() -> None:
    odr_map = build_demo_map()

    print(f"roads: {[road.id for road in odr_map.getRoads()]}")
    print(f"junctions: {[junction.id for junction in odr_map.getJunctions()]}")

    road = odr_map.getRoad("1")
    lane_section = road.get_lanesection(5.0)
    right_lane = lane_section.get_lane(-1)
    print(f"road 1 start xyz: {road.ref_line.get_xyz(0.0)}")
    print(f"right lane width at s=5: {right_lane.lane_width.get(5.0)}")
    print(f"right lane roadmarks: {[mark.type for mark in right_lane.get_roadmarks(0.0, road.length)]}")
    print(f"road objects: {[obj.id for obj in road.get_road_objects()]}")
    print(f"road signals: {[signal.id for signal in road.get_road_signals()]}")

    mesh = odr_map.getRoadNetworkMesh(eps=5.0)
    print(f"lane mesh vertices: {len(mesh.lanes_mesh.vertices)}")

    output_file = Path(__file__).with_name("tutorial_created.xodr")
    odr_map.saveXodr(output_file)
    print(f"saved: {output_file}")


if __name__ == "__main__":
    main()

