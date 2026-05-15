'''
##############################################################
# Created Date: Thursday, April 30th 2026
# Contact Info: luoxiangyong01@gmail.com
# Author/Copyright: Mr. Xiangyong Luo
##############################################################
'''

from __future__ import annotations

import json
import math
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
from typing import Any
from urllib.parse import urlparse
import webbrowser
import xml.etree.ElementTree as ET

from ..__xodr_reader import _float, _int, readXodr


WEB_DIR = Path(__file__).resolve().parent
INDEX_HTML = WEB_DIR / "index.html"
REPO_ROOT = WEB_DIR.parents[1]
DEFAULT_XODR = WEB_DIR / "data.xodr"
_ACTIVE_SERVERS: list[ThreadingHTTPServer] = []

__all__ = ["xodr_web_viewer", "run_server"]


def _as_lon_lat(odr_map: Any, x: float, y: float) -> tuple[float, float]:
    lon, lat = odr_map.convertXY2LonLat(x, y)
    return (float(lon), float(lat))


def _as_xy(odr_map: Any, lon: float, lat: float) -> tuple[float, float]:
    x, y = odr_map.convertLonLat2XY(lon, lat)
    return (float(x), float(y))


def _road_feature(odr_map: Any, road: Any, eps: float = 2.0) -> dict[str, Any]:
    s_values = road.ref_line.approximate_linear(eps, 0.0, road.length)
    coordinates: list[list[float]] = []
    local_xy: list[list[float]] = []
    for s in s_values:
        x, y, _ = road.ref_line.get_xyz(s)
        lon, lat = _as_lon_lat(odr_map, x, y)
        coordinates.append([lon, lat])
        local_xy.append([x, y])

    return {
        "type": "Feature",
        "id": road.id,
        "properties": {
            "feature_type": "road",
            "road_id": road.id,
            "name": road.name,
            "junction": road.junction,
            "length": road.length,
            "editable": True,
            "source": "pyopendrive",
        },
        "geometry": {
            "type": "LineString",
            "coordinates": coordinates,
        },
        "bbox_xy": local_xy,
    }


def _feature_collection(features: list[dict[str, Any]]) -> dict[str, Any]:
    bbox = None
    if features:
        coords: list[list[float]] = []
        for feature in features:
            geometry = feature.get("geometry", {})
            if geometry.get("type") == "LineString":
                coords.extend(geometry.get("coordinates", []))
            elif geometry.get("type") == "Polygon":
                for ring in geometry.get("coordinates", []):
                    coords.extend(ring)
            elif geometry.get("type") == "Point":
                coords.append(geometry.get("coordinates", []))
        coords = [coord for coord in coords if len(coord) >= 2]
        if coords:
            lons = [coord[0] for coord in coords]
            lats = [coord[1] for coord in coords]
            bbox = [min(lons), min(lats), max(lons), max(lats)]
    return {
        "type": "FeatureCollection",
        "features": features,
        "bbox": bbox,
    }


def _map_to_geojson(odr_map: Any) -> dict[str, Any]:
    return _feature_collection(
        [
            _road_feature(odr_map, road)
            for road in odr_map.getRoads()
            if road.ref_line.s0_to_geometry
        ]
    )


def _lane_feature(
    odr_map: Any,
    road: Any,
    lane: Any,
    *,
    section_end: float,
    lane_successors: dict[str, list[str]],
    lane_predecessors: dict[str, list[str]],
    eps: float = 2.0,
) -> dict[str, Any] | None:
    if lane.id == 0:
        return None

    mesh = road.get_lane_mesh(lane, eps)
    if len(mesh.vertices) < 4:
        return None

    outer = [_as_lon_lat(odr_map, x, y) for x, y, _ in mesh.vertices[0::2]]
    inner = [_as_lon_lat(odr_map, x, y) for x, y, _ in mesh.vertices[1::2]]
    ring = [[lon, lat] for lon, lat in outer]
    ring.extend([[lon, lat] for lon, lat in reversed(inner)])
    if ring and ring[0] != ring[-1]:
        ring.append(ring[0])
    if len(ring) < 4:
        return None

    lane_key = lane.key.to_string()
    predecessor_keys = list(dict.fromkeys(lane_predecessors.get(lane_key, [])))
    successor_keys = list(dict.fromkeys(lane_successors.get(lane_key, [])))
    return {
        "type": "Feature",
        "id": lane_key,
        "properties": {
            "feature_type": "lane",
            "road_id": road.id,
            "road_name": road.name,
            "junction": road.junction,
            "lane_key": lane_key,
            "lane_id": lane.id,
            "lane_type": lane.type,
            "level": lane.level,
            "lanesection_s0": lane.lanesection_s0,
            "lanesection_end": section_end,
            "predecessor": lane.predecessor,
            "successor": lane.successor,
            "predecessor_keys": predecessor_keys,
            "successor_keys": successor_keys,
            "source": "pyopendrive",
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [ring],
        },
    }


def _map_to_lane_geojson(odr_map: Any) -> dict[str, Any]:
    features: list[dict[str, Any]] = []
    routing_graph = odr_map.getRoutingGraph()
    lane_successors = {
        lane_key.to_string(): [successor.to_string() for successor, _ in successors]
        for lane_key, successors in routing_graph.lane_key_to_successors.items()
    }
    lane_predecessors = {
        lane_key.to_string(): [predecessor.to_string() for predecessor, _ in predecessors]
        for lane_key, predecessors in routing_graph.lane_key_to_predecessors.items()
    }
    for road in odr_map.getRoads():
        if not road.ref_line.s0_to_geometry:
            continue
        for section in road.get_lanesections():
            section_end = road.get_lanesection_end(section)
            for lane in section.get_lanes():
                feature = _lane_feature(
                    odr_map,
                    road,
                    lane,
                    section_end=section_end,
                    lane_successors=lane_successors,
                    lane_predecessors=lane_predecessors,
                )
                if feature is not None:
                    features.append(feature)
    return _feature_collection(features)


def _meter_box(lon: float, lat: float, size_m: float) -> list[list[float]]:
    half = max(0.25, size_m * 0.5)
    lat_scale = 110_540.0
    lon_scale = max(1e-9, 111_320.0 * math.cos(math.radians(lat)))
    dlon = half / lon_scale
    dlat = half / lat_scale
    return [
        [lon - dlon, lat - dlat],
        [lon + dlon, lat - dlat],
        [lon + dlon, lat + dlat],
        [lon - dlon, lat + dlat],
        [lon - dlon, lat - dlat],
    ]


def _road_heading(road: Any, s: float) -> float:
    dx, dy, _ = road.ref_line.get_grad(s)
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return math.atan2(dy, dx)


def _road_st_from_lon_lat(odr_map: Any, road: Any, lon: Any, lat: Any) -> tuple[float, float]:
    x, y = _as_xy(odr_map, float(lon), float(lat))
    s = road.ref_line.match(x, y)
    ref_x, ref_y, _ = road.ref_line.get_xyz(s)
    grad_x, grad_y, _ = road.ref_line.get_grad(s)
    length = math.hypot(grad_x, grad_y)
    if length <= 1e-9:
        return (float(s), 0.0)
    normal_x = -grad_y / length
    normal_y = grad_x / length
    t = (x - ref_x) * normal_x + (y - ref_y) * normal_y
    return (float(s), float(t))


def _oriented_box(
    odr_map: Any,
    center_x: float,
    center_y: float,
    heading: float,
    length_m: float,
    width_m: float,
) -> list[list[float]]:
    half_length = max(0.2, length_m * 0.5)
    half_width = max(0.2, width_m * 0.5)
    along = (math.cos(heading), math.sin(heading))
    across = (-math.sin(heading), math.cos(heading))
    corners_xy = [
        (
            center_x + along[0] * half_length + across[0] * half_width,
            center_y + along[1] * half_length + across[1] * half_width,
        ),
        (
            center_x + along[0] * half_length - across[0] * half_width,
            center_y + along[1] * half_length - across[1] * half_width,
        ),
        (
            center_x - along[0] * half_length - across[0] * half_width,
            center_y - along[1] * half_length - across[1] * half_width,
        ),
        (
            center_x - along[0] * half_length + across[0] * half_width,
            center_y - along[1] * half_length + across[1] * half_width,
        ),
    ]
    ring = [[*_as_lon_lat(odr_map, x, y)] for x, y in corners_xy]
    ring.append(ring[0])
    return ring


def _validity_properties(validities: Any) -> list[dict[str, int]]:
    return [
        {"from_lane": validity.from_lane, "to_lane": validity.to_lane}
        for validity in validities
    ]


def _float_or(value: Any, fallback: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    if not math.isfinite(number):
        return fallback
    return number


def _signal_feature(odr_map: Any, road: Any, signal: Any) -> dict[str, Any]:
    signal_s = _float_or(signal.s0, 0.0)
    signal_t = _float_or(signal.t0, 0.0)
    z_offset = _float_or(signal.zOffset, 0.0)
    signal_width = _float_or(signal.width, 0.6)
    signal_height = _float_or(signal.height, 1.0)
    x, y, z = road.get_xyz(signal_s, signal_t, z_offset)
    lon, lat = _as_lon_lat(odr_map, x, y)
    footprint = _meter_box(lon, lat, max(signal_width, 0.6))
    signal_key = f"{road.id}/{signal.id}"
    return {
        "type": "Feature",
        "id": signal_key,
        "properties": {
            "feature_type": "signal",
            "signal_key": signal_key,
            "signal_id": signal.id,
            "road_id": road.id,
            "road_name": road.name,
            "name": signal.name,
            "lon": lon,
            "lat": lat,
            "s": signal_s,
            "t": signal_t,
            "z": z,
            "dynamic": signal.is_dynamic,
            "zOffset": z_offset,
            "value": signal.value,
            "height": signal_height,
            "width": signal_width,
            "hOffset": signal.hOffset,
            "pitch": signal.pitch,
            "roll": signal.roll,
            "orientation": signal.orientation,
            "country": signal.country,
            "type": signal.type,
            "subtype": signal.subtype,
            "unit": signal.unit,
            "text": signal.text,
            "validities": _validity_properties(signal.lane_validities),
            "extrusion_base": max(0.0, z_offset),
            "extrusion_height": max(z_offset + max(signal_height, 1.0), 1.0),
            "source": "pyopendrive",
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [footprint],
        },
    }


def _is_signal_support_object(obj: Any) -> bool:
    name = str(getattr(obj, "name", "") or "").lower()
    obj_type = str(getattr(obj, "type", "") or "").lower()
    return any(
        token in name or token in obj_type
        for token in ("signal_post", "signal_mastarm", "signpost", "mastarm")
    )


def _signal_support_role(obj: Any) -> str:
    name = str(getattr(obj, "name", "") or "").lower()
    obj_type = str(getattr(obj, "type", "") or "").lower()
    label = f"{name} {obj_type}"
    if "mastarm" in label or "mast_arm" in label or "mast arm" in label:
        return "mastarm"
    if "post" in label:
        return "post"
    return "support"


def _signal_support_object_feature(
    odr_map: Any,
    road: Any,
    obj: Any,
) -> dict[str, Any]:
    obj_s = _float_or(obj.s0, 0.0)
    obj_t = _float_or(obj.t0, 0.0)
    z_offset = _float_or(obj.z0, 0.0)
    obj_length = _float_or(obj.length, 0.0)
    obj_width = _float_or(obj.width, 0.0)
    obj_radius = _float_or(obj.radius, 0.0)
    obj_height = _float_or(obj.height, 1.0)
    obj_hdg = _float_or(obj.hdg, 0.0)
    x, y, z = road.get_xyz(obj_s, obj_t, z_offset)
    lon, lat = _as_lon_lat(odr_map, x, y)
    heading = _road_heading(road, obj_s) + obj_hdg
    footprint = _oriented_box(
        odr_map,
        x,
        y,
        heading,
        max(obj_length, obj_radius * 2.0, 0.4),
        max(obj_width, obj_radius * 2.0, 0.4),
    )
    obj_key = f"{road.id}/{obj.id}"
    role = _signal_support_role(obj)
    return {
        "type": "Feature",
        "id": obj_key,
        "properties": {
            "feature_type": "signal_object",
            "object_role": role,
            "object_key": obj_key,
            "object_id": obj.id,
            "road_id": road.id,
            "road_name": road.name,
            "name": obj.name,
            "lon": lon,
            "lat": lat,
            "s": obj_s,
            "t": obj_t,
            "z": z,
            "zOffset": z_offset,
            "length": obj_length,
            "validLength": _float_or(obj.valid_length, 0.0),
            "width": obj_width,
            "radius": obj_radius,
            "height": obj_height,
            "hdg": obj_hdg,
            "map_heading": heading,
            "pitch": obj.pitch,
            "roll": obj.roll,
            "orientation": obj.orientation,
            "type": obj.type,
            "subtype": obj.subtype,
            "dynamic": obj.is_dynamic,
            "validities": _validity_properties(obj.lane_validities),
            "extrusion_base": max(0.0, z_offset),
            "extrusion_height": max(z_offset + max(obj_height, 0.4), 0.4),
            "source": "pyopendrive",
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [footprint],
        },
    }


def _map_to_signal_geojson(odr_map: Any) -> dict[str, Any]:
    features: list[dict[str, Any]] = []
    support_features: list[dict[str, Any]] = []
    for road in odr_map.getRoads():
        for obj in road.get_road_objects():
            if _is_signal_support_object(obj):
                support_features.append(_signal_support_object_feature(odr_map, road, obj))
    features.extend(support_features)
    for road in odr_map.getRoads():
        for signal in road.get_road_signals():
            if not support_features or str(signal.name).startswith("User_"):
                features.append(_signal_feature(odr_map, road, signal))
    return _feature_collection(features)


def _set_plan_view_to_lines(road_node: ET.Element, xy: list[tuple[float, float]]) -> None:
    plan_view = road_node.find("planView")
    if plan_view is None:
        plan_view = ET.SubElement(road_node, "planView")
    for child in list(plan_view):
        plan_view.remove(child)

    cumulative_s = 0.0
    for start, end in zip(xy, xy[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            continue
        geom = ET.SubElement(
            plan_view,
            "geometry",
            {
                "s": f"{cumulative_s:.8f}",
                "x": f"{start[0]:.8f}",
                "y": f"{start[1]:.8f}",
                "hdg": f"{math.atan2(dy, dx):.12f}",
                "length": f"{length:.8f}",
            },
        )
        ET.SubElement(geom, "line")
        cumulative_s += length

    road_node.set("length", f"{cumulative_s:.8f}")


def _new_road_node(road_id: str, name: str, xy: list[tuple[float, float]]) -> ET.Element:
    road_node = ET.Element(
        "road",
        {
            "name": name,
            "length": "0.00000000",
            "id": road_id,
            "junction": "-1",
        },
    )
    _set_plan_view_to_lines(road_node, xy)
    lanes = ET.SubElement(road_node, "lanes")
    section = ET.SubElement(lanes, "laneSection", {"s": "0.00000000"})
    center = ET.SubElement(section, "center")
    ET.SubElement(center, "lane", {"id": "0", "type": "none", "level": "false"})
    return road_node


def _fmt(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        if isinstance(value, float) and not math.isfinite(value):
            return "0"
        return f"{float(value):.12g}"
    return str(value)


def _set_attrs(node: ET.Element, attrs: dict[str, Any]) -> None:
    for key, value in attrs.items():
        if value is None:
            continue
        node.set(key, _fmt(value))


def _lane_parent(section_node: ET.Element, lane_id: int) -> ET.Element:
    side_name = "left" if lane_id > 0 else "right"
    side = section_node.find(side_name)
    if side is None:
        side = ET.SubElement(section_node, side_name)
    return side


def _find_lane_section_node(road_node: ET.Element, s0: float) -> ET.Element | None:
    sections = road_node.findall("./lanes/laneSection")
    if not sections:
        return None
    return min(
        sections,
        key=lambda node: abs(_float(node, "s", 0.0) - s0),
    )


def _set_lane_link(lane_node: ET.Element, tag: str, lane_id: Any) -> None:
    link = lane_node.find("link")
    if link is None:
        link = ET.SubElement(lane_node, "link")
    for child in list(link.findall(tag)):
        link.remove(child)
    try:
        linked_id = int(float(lane_id))
    except (TypeError, ValueError):
        linked_id = 0
    if linked_id != 0:
        ET.SubElement(link, tag, {"id": str(linked_id)})
    if len(list(link)) == 0:
        lane_node.remove(link)


def _apply_lane_geojson_edits(odr_map: Any, lane_geojson: dict[str, Any]) -> None:
    features = lane_geojson.get("features") or []
    if not features:
        return

    root = odr_map.root
    lane_keys: set[tuple[str, float, int]] = set()
    for feature in features:
        props = feature.get("properties") or {}
        if props.get("feature_type") != "lane":
            continue
        road_id = str(props.get("road_id") or "")
        lane_id = int(float(props.get("lane_id") or 0))
        section_s0 = float(props.get("lanesection_s0") or 0.0)
        if not road_id or lane_id == 0:
            continue

        road_node = root.find(f"./road[@id='{road_id}']")
        if road_node is None:
            continue
        section_node = _find_lane_section_node(road_node, section_s0)
        if section_node is None:
            lanes_node = road_node.find("lanes")
            if lanes_node is None:
                lanes_node = ET.SubElement(road_node, "lanes")
            section_node = ET.SubElement(lanes_node, "laneSection", {"s": _fmt(section_s0)})

        lane_keys.add((road_id, section_s0, lane_id))
        lane_node = section_node.find(f".//lane[@id='{lane_id}']")
        if lane_node is None:
            lane_node = ET.SubElement(_lane_parent(section_node, lane_id), "lane")
            ET.SubElement(
                lane_node,
                "width",
                {"sOffset": "0", "a": _fmt(props.get("width", 3.5)), "b": "0", "c": "0", "d": "0"},
            )

        _set_attrs(
            lane_node,
            {
                "id": lane_id,
                "type": props.get("lane_type") or props.get("type") or lane_node.get("type", "driving"),
                "level": props.get("level", lane_node.get("level", "false")),
            },
        )
        if "predecessor" in props:
            _set_lane_link(lane_node, "predecessor", props.get("predecessor"))
        if "successor" in props:
            _set_lane_link(lane_node, "successor", props.get("successor"))

    for road_node in root.findall("road"):
        road_id = road_node.get("id", "")
        for section_node in road_node.findall("./lanes/laneSection"):
            section_s0 = _float(section_node, "s", 0.0)
            for side_name in ("left", "right"):
                side = section_node.find(side_name)
                if side is None:
                    continue
                for lane_node in list(side.findall("lane")):
                    lane_id = _int(lane_node, "id", 0)
                    if lane_id != 0 and (road_id, section_s0, lane_id) not in lane_keys:
                        side.remove(lane_node)


def _objects_node(road_node: ET.Element) -> ET.Element:
    node = road_node.find("objects")
    if node is None:
        node = ET.SubElement(road_node, "objects")
    return node


def _signals_node(road_node: ET.Element) -> ET.Element:
    node = road_node.find("signals")
    if node is None:
        node = ET.SubElement(road_node, "signals")
    return node


def _apply_signal_geojson_edits(odr_map: Any, signal_geojson: dict[str, Any]) -> None:
    features = signal_geojson.get("features") or []
    if not features:
        return

    root = odr_map.root
    seen_objects: set[tuple[str, str]] = set()
    seen_signals: set[tuple[str, str]] = set()
    has_object_features = False
    has_signal_features = False

    for feature in features:
        props = feature.get("properties") or {}
        road_id = str(props.get("road_id") or "")
        if not road_id:
            continue
        road_node = root.find(f"./road[@id='{road_id}']")
        if road_node is None:
            continue
        feature_road = odr_map.getRoad(road_id)
        if props.get("lon") is not None and props.get("lat") is not None:
            try:
                props = {
                    **props,
                    **dict(zip(("s", "t"), _road_st_from_lon_lat(odr_map, feature_road, props["lon"], props["lat"]))),
                }
            except Exception:
                pass

        if props.get("feature_type") == "signal_object":
            has_object_features = True
            object_id = str(props.get("object_id") or props.get("object_key") or "")
            if not object_id:
                continue
            seen_objects.add((road_id, object_id))
            obj_node = road_node.find(f"./objects/object[@id='{object_id}']")
            if obj_node is None:
                obj_node = ET.SubElement(_objects_node(road_node), "object")
            _set_attrs(
                obj_node,
                {
                    "id": object_id,
                    "name": props.get("name") or ("Signal_MastArm_15ft" if props.get("object_role") == "mastarm" else "Signal_Post_30ft"),
                    "s": props.get("s", 0.0),
                    "t": props.get("t", 0.0),
                    "zOffset": props.get("zOffset", 0.0),
                    "hdg": props.get("hdg", 0.0),
                    "roll": props.get("roll", 0.0),
                    "pitch": props.get("pitch", 0.0),
                    "orientation": props.get("orientation", "+"),
                    "type": props.get("type", "-1"),
                    "subtype": props.get("subtype"),
                    "dynamic": "yes" if props.get("dynamic") else "no",
                    "height": props.get("height", 3.0),
                    "width": props.get("width", 0.5),
                    "length": props.get("length", 0.5),
                },
            )
        elif props.get("feature_type") == "signal":
            has_signal_features = True
            signal_id = str(props.get("signal_id") or props.get("signal_key") or "")
            if not signal_id:
                continue
            seen_signals.add((road_id, signal_id))
            sig_node = road_node.find(f"./signals/signal[@id='{signal_id}']")
            if sig_node is None:
                sig_node = ET.SubElement(_signals_node(road_node), "signal")
            _set_attrs(
                sig_node,
                {
                    "id": signal_id,
                    "name": props.get("name") or "User_Signal_Head",
                    "s": props.get("s", 0.0),
                    "t": props.get("t", 0.0),
                    "zOffset": props.get("zOffset", 0.0),
                    "hOffset": props.get("hOffset", 0.0),
                    "roll": props.get("roll", 0.0),
                    "pitch": props.get("pitch", 0.0),
                    "orientation": props.get("orientation", "+"),
                    "dynamic": "yes" if props.get("dynamic", True) else "no",
                    "country": props.get("country", "OpenDRIVE"),
                    "type": props.get("type", "1000001"),
                    "subtype": props.get("subtype", "-1"),
                    "value": props.get("value", -1.0),
                    "unit": props.get("unit", ""),
                    "text": props.get("text", ""),
                    "height": props.get("height", 1.0),
                    "width": props.get("width", 0.5),
                },
            )

    if has_object_features:
        for road_node in root.findall("road"):
            road_id = road_node.get("id", "")
            objects = road_node.find("objects")
            if objects is None:
                continue
            for obj_node in list(objects.findall("object")):
                name = obj_node.get("name", "").lower()
                if _is_signal_support_object(type("Obj", (), {"name": name, "type": obj_node.get("type", "")})()):
                    if (road_id, obj_node.get("id", "")) not in seen_objects:
                        objects.remove(obj_node)
    if has_signal_features:
        for road_node in root.findall("road"):
            road_id = road_node.get("id", "")
            signals = road_node.find("signals")
            if signals is None:
                continue
            for sig_node in list(signals.findall("signal")):
                if (road_id, sig_node.get("id", "")) not in seen_signals:
                    signals.remove(sig_node)


def _apply_geojson_edits(odr_map: Any, geojson: dict[str, Any]) -> None:
    root = odr_map.root
    existing = {road.get("id"): road for road in root.findall("road")}
    seen_ids: set[str] = set()
    next_id = 1

    for feature in geojson.get("features", []):
        if feature.get("geometry", {}).get("type") != "LineString":
            continue
        coords = feature.get("geometry", {}).get("coordinates") or []
        if len(coords) < 2:
            continue

        props = feature.get("properties") or {}
        road_id = str(props.get("road_id") or feature.get("id") or "")
        if not road_id:
            while str(next_id) in existing or str(next_id) in seen_ids:
                next_id += 1
            road_id = str(next_id)
        seen_ids.add(road_id)

        xy = [_as_xy(odr_map, float(lon), float(lat)) for lon, lat, *_ in coords]
        road_node = existing.get(road_id)
        if road_node is None:
            road_node = _new_road_node(road_id, str(props.get("name") or ""), xy)
            root.append(road_node)
            existing[road_id] = road_node
        else:
            _set_plan_view_to_lines(road_node, xy)

    for road_id, road_node in list(existing.items()):
        if road_id not in seen_ids:
            root.remove(road_node)


class _ViewerState:
    def __init__(self, default_xodr: Path | None = None):
        self.lock = threading.Lock()
        self.tmp_dir = tempfile.TemporaryDirectory(prefix="pyopendrive_web_")
        self.current_file: Path | None = None
        self.current_map: Any | None = None
        if default_xodr is not None and default_xodr.exists():
            self.load_path(default_xodr)

    def load_path(self, xodr_path: Path) -> dict[str, Any]:
        self.current_file = Path(xodr_path)
        self.current_map = readXodr(self.current_file)
        return self.as_response()

    def load_text(self, file_text: str, filename: str = "network.xodr") -> dict[str, Any]:
        target = Path(self.tmp_dir.name) / Path(filename).name
        target.write_text(file_text, encoding="utf-8")
        return self.load_path(target)

    def as_response(self) -> dict[str, Any]:
        if self.current_map is None:
            raise RuntimeError("No OpenDRIVE map is loaded.")
        return {
            "filename": self.current_file.name if self.current_file else "network.xodr",
            "geojson": _map_to_geojson(self.current_map),
            "lane_geojson": _map_to_lane_geojson(self.current_map),
            "signal_geojson": _map_to_signal_geojson(self.current_map),
            "proj4": self.current_map.getGeoProj(),
            "boundary": self.current_map.getBoundary(),
        }

    def save_geojson(
        self,
        geojson: dict[str, Any],
        lane_geojson: dict[str, Any] | None = None,
        signal_geojson: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self.current_map is None:
            raise RuntimeError("No OpenDRIVE map is loaded.")
        _apply_geojson_edits(self.current_map, geojson)
        if lane_geojson is not None:
            _apply_lane_geojson_edits(self.current_map, lane_geojson)
        if signal_geojson is not None:
            _apply_signal_geojson_edits(self.current_map, signal_geojson)
        target = Path(self.tmp_dir.name) / "edited_network.xodr"
        self.current_map.saveXodr(target)
        self.current_map = readXodr(target)
        self.current_file = target
        return {
            **self.as_response(),
            "xodr": target.read_text(encoding="utf-8"),
            "saved_filename": target.name,
        }


class _OpenDriveViewerHandler(SimpleHTTPRequestHandler):
    state: _ViewerState

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, directory=str(WEB_DIR), **kwargs)

    def _json_response(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        return json.loads(body) if body else {}

    def do_GET(self) -> None:
        if urlparse(self.path).path == "/api/network":
            try:
                with self.state.lock:
                    payload = self.state.as_response()
                self._json_response(HTTPStatus.OK, payload)
            except Exception as exc:
                self._json_response(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    {"error": str(exc)},
                )
            return
        super().do_GET()

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            payload = self._read_json()
            with self.state.lock:
                if path == "/api/load-xodr":
                    response = self.state.load_text(
                        str(payload.get("xodr") or ""),
                        str(payload.get("filename") or "network.xodr"),
                    )
                elif path == "/api/save-xodr":
                    response = self.state.save_geojson(
                        payload.get("geojson") or {},
                        payload.get("lane_geojson"),
                        payload.get("signal_geojson"),
                    )
                else:
                    self._json_response(HTTPStatus.NOT_FOUND, {"error": "Not found"})
                    return
            self._json_response(HTTPStatus.OK, response)
        except Exception as exc:
            self._json_response(HTTPStatus.BAD_REQUEST, {"error": str(exc)})


def run_server(
    host: str = "127.0.0.1",
    port: int = 8765,
    *,
    open_browser: bool = True,
    default_xodr: str | Path | None = DEFAULT_XODR,
) -> tuple[ThreadingHTTPServer, str]:
    """Run the pyopendrive MapLibre web viewer server."""
    state = _ViewerState(Path(default_xodr) if default_xodr is not None else None)

    class Handler(_OpenDriveViewerHandler):
        pass

    Handler.state = state
    server = ThreadingHTTPServer((host, port), Handler)
    url = f"http://{host}:{server.server_port}/index.html"
    if open_browser:
        webbrowser.open(url)
    return server, url


def xodr_web_viewer(
    index_html: str | Path = INDEX_HTML,
    *,
    host: str = "127.0.0.1",
    port: int = 8765,
    default_xodr: str | Path | None = DEFAULT_XODR,
) -> str:
    """Open the bundled MapLibre viewer in the default browser.

    Args:
        index_html: Kept for backward compatibility. Static file opening is no
            longer used because pyopendrive conversion runs through a local API.
        host: Local interface to bind.
        port: Local port to bind. Use ``0`` to choose any free port.
        default_xodr: Optional OpenDRIVE file loaded at startup.

    Returns:
        The viewer URL.
    """
    _ = Path(index_html).resolve()
    server, url = run_server(
        host=host,
        port=port,
        open_browser=True,
        default_xodr=default_xodr,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    _ACTIVE_SERVERS.append(server)
    return url
