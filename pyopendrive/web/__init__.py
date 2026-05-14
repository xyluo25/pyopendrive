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

from ..__xodr_reader import readXodr


WEB_DIR = Path(__file__).resolve().parent
INDEX_HTML = WEB_DIR / "index.html"
REPO_ROOT = WEB_DIR.parents[1]
DEFAULT_XODR = REPO_ROOT / "datasets" / "chatt.xodr"
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


def _map_to_geojson(odr_map: Any) -> dict[str, Any]:
    features = [
        _road_feature(odr_map, road)
        for road in odr_map.getRoads()
        if road.ref_line.s0_to_geometry
    ]
    bbox = None
    if features:
        lons = [
            coord[0]
            for feature in features
            for coord in feature["geometry"]["coordinates"]
        ]
        lats = [
            coord[1]
            for feature in features
            for coord in feature["geometry"]["coordinates"]
        ]
        bbox = [min(lons), min(lats), max(lons), max(lats)]
    return {
        "type": "FeatureCollection",
        "features": features,
        "bbox": bbox,
    }


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
            "proj4": self.current_map.getGeoProj(),
            "boundary": self.current_map.getBoundary(),
        }

    def save_geojson(self, geojson: dict[str, Any]) -> dict[str, Any]:
        if self.current_map is None:
            raise RuntimeError("No OpenDRIVE map is loaded.")
        _apply_geojson_edits(self.current_map, geojson)
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
                    response = self.state.save_geojson(payload.get("geojson") or {})
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
