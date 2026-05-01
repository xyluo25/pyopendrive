from __future__ import annotations

import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest

# Add system path for imports from pyopendrive package
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pyopendrive as odr


SYNTHETIC_XODR = """<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="unit">
    <geoReference>+proj=tmerc +lat_0=0</geoReference>
  </header>
  <road name="main" length="40" id="1" junction="-1" rule="RHT">
    <link>
      <successor elementType="junction" elementId="10"/>
      <neighbor elementId="2" side="right" direction="same"/>
    </link>
    <type s="0" type="town">
      <speed max="35" unit="mph"/>
    </type>
    <planView>
      <geometry s="0" x="100" y="50" hdg="0" length="10"><line/></geometry>
      <geometry s="10" x="110" y="50" hdg="0" length="10"><arc curvature="0.02"/></geometry>
      <geometry s="20" x="119.93" y="51.0" hdg="0.2" length="10"><spiral curvStart="0.0" curvEnd="0.0"/></geometry>
      <geometry s="30" x="129.73" y="52.99" hdg="0.2" length="10">
        <paramPoly3 aU="0" bU="10" cU="0" dU="0" aV="0" bV="0" cV="0" dV="0" pRange="normalized"/>
      </geometry>
    </planView>
    <elevationProfile>
      <elevation s="0" a="0" b="0.01" c="0" d="0"/>
    </elevationProfile>
    <lateralProfile>
      <superelevation s="0" a="0.01" b="0" c="0" d="0"/>
      <crossfall s="0" side="both" a="0.02" b="0" c="0" d="0"/>
      <crossfall s="20" side="left" a="0.03" b="0" c="0" d="0"/>
    </lateralProfile>
    <lanes>
      <laneOffset s="0" a="0.2" b="0" c="0" d="0"/>
      <laneSection s="0">
        <left>
          <lane id="1" type="driving" level="false">
            <link><successor id="1"/></link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
            <height sOffset="0" inner="0.1" outer="0.2"/>
            <roadMark sOffset="0" type="solid" weight="standard" color="white" laneChange="none"/>
          </lane>
        </left>
        <center>
          <lane id="0" type="none" level="false">
            <roadMark sOffset="0" type="none"/>
          </lane>
        </center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><successor id="-1"/></link>
            <width sOffset="0" a="3.25" b="0" c="0" d="0"/>
            <roadMark sOffset="0" type="broken" weight="bold">
              <type name="dash" width="0.15">
                <line length="2" space="1" tOffset="0.1" sOffset="0" rule="none"/>
              </type>
            </roadMark>
          </lane>
        </right>
      </laneSection>
    </lanes>
    <objects>
      <object id="obj1" s="5" t="1" zOffset="0.2" length="2" validLength="3" width="1" height="2"
              hdg="0.1" pitch="0.2" roll="0.3" type="pole" name="Pole" orientation="+" subtype="sign" dynamic="yes">
        <repeat s="6" length="5" distance="2" tStart="1" tEnd="2" widthStart="1" widthEnd="2"
                heightStart="2" heightEnd="3" zOffsetStart="0.1" zOffsetEnd="0.2"/>
        <outlines>
          <outline id="1" fillType="grass" laneType="driving" outer="true" closed="true">
            <cornerLocal id="1" u="0" v="0" z="0" height="1"/>
            <cornerRoad id="2" s="6" t="1" dz="0" height="1"/>
          </outline>
        </outlines>
        <validity fromLane="-1" toLane="1"/>
      </object>
    </objects>
    <signals>
      <signal id="sig1" name="Speed" s="4" t="-1" dynamic="true" zOffset="1" value="35"
              height="1" width="0.5" hOffset="0.1" pitch="0.2" roll="0.3" orientation="+"
              country="US" type="274" subtype="35" unit="mph" text="35">
        <validity fromLane="-1" toLane="-1"/>
      </signal>
    </signals>
  </road>
  <road name="connector" length="12" id="2" junction="10" rule="LHT">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
    </link>
    <planView>
      <geometry s="0" x="140" y="55" hdg="0.2" length="12"><spiral curvStart="0.01" curvEnd="0.02"/></geometry>
    </planView>
    <lanes>
      <laneSection s="0">
        <center><lane id="0" type="none"/></center>
        <right>
          <lane id="-1" type="driving">
            <link><predecessor id="-1"/></link>
            <width sOffset="0" a="3" b="0" c="0" d="0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <junction name="junction" id="10">
    <connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">
      <laneLink from="-1" to="-1"/>
    </connection>
    <priority high="1" low="2"/>
    <controller id="ctrl" type="signal" sequence="1"/>
  </junction>
</OpenDRIVE>
"""


@pytest.fixture()
def synthetic_file(tmp_path: Path) -> Path:
    xodr = tmp_path / "synthetic.xodr"
    xodr.write_text(SYNTHETIC_XODR, encoding="utf-8")
    return xodr


@pytest.fixture()
def synthetic_map(synthetic_file: Path) -> odr.OpenDriveMap:
    return odr.OpenDriveMap(str(synthetic_file))


def assert_vec_finite(vec: tuple[float, ...]) -> None:
    assert all(math.isfinite(v) for v in vec)


def test_synthetic_map_parses_roads_junctions_and_metadata(
    synthetic_map: odr.OpenDriveMap,
) -> None:
    assert synthetic_map.proj4.startswith("+proj=tmerc")
    assert len(synthetic_map.get_roads()) == 2
    assert len(synthetic_map.get_junctions()) == 1

    road = synthetic_map.get_road("1")
    assert road.name == "main"
    assert road.successor.type == "junction"
    assert road.neighbors[0].id == "2"
    assert road.s_to_type[0.0] == "town"
    assert road.s_to_speed[0.0].max == "35"
    assert len(road.ref_line.get_geometries()) == 4

    connector = synthetic_map.get_road("2")
    assert connector.left_hand_traffic is True
    assert connector.predecessor.id == "1"

    junction = synthetic_map.get_junction("10")
    assert junction.id_to_connection["0"].lane_links[0].from_lane == -1
    assert junction.priorities[0].high == "1"
    assert junction.id_to_controller["ctrl"].sequence == 1


def test_open_drive_map_save_xodr_preserves_loaded_xml(
    synthetic_map: odr.OpenDriveMap,
    tmp_path: Path,
) -> None:
    saved = synthetic_map.save_xodr(tmp_path / "nested" / "saved.xodr")

    assert saved == tmp_path / "nested" / "saved.xodr"
    assert saved.exists()

    root = ET.parse(saved).getroot()
    assert root.tag == "OpenDRIVE"
    assert root.find("./road[@id='1']") is not None
    assert root.find("./junction[@id='10']") is not None

    reloaded = odr.load(saved)
    assert len(reloaded.get_roads()) == len(synthetic_map.get_roads())
    assert len(reloaded.get_junctions()) == len(synthetic_map.get_junctions())


def test_lane_queries_roadmarks_and_surface_points(
    synthetic_map: odr.OpenDriveMap,
) -> None:
    road = synthetic_map.get_road("1")
    section = road.get_lanesection(1.0)
    lanes = section.get_lanes()
    assert [lane.id for lane in lanes] == [-1, 0, 1]

    left_lane = section.get_lane(1)
    right_lane = section.get_lane(-1)
    assert left_lane.outer_border.get(0.0) == pytest.approx(3.7)
    assert right_lane.outer_border.get(0.0) == pytest.approx(-3.05)
    assert section.get_lane_id(1.0, 3.0) == 1
    assert section.get_lane(1.0, -1.0).id == -1

    left_marks = left_lane.get_roadmarks(0.0, road.length)
    right_marks = right_lane.get_roadmarks(0.0, 7.0)
    assert left_marks[0].type == "solid"
    assert {mark.type for mark in right_marks} == {"brokendash"}

    assert_vec_finite(road.ref_line.get_xyz(5.0))
    assert_vec_finite(road.ref_line.get_grad(5.0))
    assert_vec_finite(road.get_xyz(5.0, 1.0, 0.5))
    assert_vec_finite(road.get_surface_pt(5.0, 1.0))
    assert road.crossfall.get_crossfall(25.0, on_left_side=False) == 0.0

    border = road.get_lane_border_line(left_lane, eps=2.0)
    assert len(border) >= 2
    assert all(len(point) == 3 for point in border)


def test_objects_signals_mesh_and_routing(synthetic_map: odr.OpenDriveMap) -> None:
    road = synthetic_map.get_road("1")
    road_object = road.id_to_object["obj1"]
    signal = road.id_to_signal["sig1"]

    assert road_object.is_dynamic is True
    assert road_object.repeats[0].distance == 2.0
    assert road_object.outlines[0].outline[0].type == "local_rel_z"
    assert road_object.lane_validities[0].from_lane == -1
    assert signal.country == "US"
    assert signal.lane_validities[0].to_lane == -1

    lane = road.get_lanesection(1.0).get_lane(1)
    lane_mesh = road.get_lane_mesh(lane, eps=2.0)
    assert len(lane_mesh.vertices) >= 4
    assert len(lane_mesh.indices) >= 6

    roadmark_mesh = road.get_roadmark_mesh(
        lane,
        lane.get_roadmarks(0.0, road.length)[0],
        eps=2.0,
    )
    assert len(roadmark_mesh.vertices) >= 4

    network_mesh = synthetic_map.get_road_network_mesh(eps=4.0)
    assert len(network_mesh.lanes_mesh.vertices) > 0
    assert len(network_mesh.roadmarks_mesh.vertices) > 0
    assert len(network_mesh.get_mesh().vertices) >= len(
        network_mesh.lanes_mesh.vertices
    )

    graph = synthetic_map.get_routing_graph()
    start = odr.LaneKey("1", 0.0, -1)
    end = odr.LaneKey("2", 0.0, -1)
    assert end in graph.get_lane_successors(start)
    assert graph.get_lane_predecessors(end)
    assert graph.shortest_path(start, end) == [start, end]


def test_centering_and_outline_z_mode(synthetic_file: Path) -> None:
    centered = odr.OpenDriveMap(
        str(synthetic_file),
        center_map=True,
        abs_z_for_for_local_road_obj_outline=True,
    )
    assert centered.x_offs > 0
    assert centered.y_offs > 0
    road_object = centered.get_road("1").id_to_object["obj1"]
    assert road_object.outlines[0].outline[0].type == "local_abs_z"


def test_geometry_and_spline_helpers() -> None:
    poly = odr.Poly3.from_odr(2.0, 1.0, 2.0, 3.0, 4.0)
    assert math.isfinite(poly.get(3.0))
    assert math.isfinite(poly.get_grad(3.0))
    assert not poly.isnan()
    assert poly.negate().get(3.0) == pytest.approx(-poly.get(3.0))
    assert len(poly.approximate_linear(0.5, 0.0, 2.0)) >= 2

    spline = odr.CubicSpline({0.0: odr.Poly3.from_odr(0.0, 1.0, 0.0, 0.0, 0.0)})
    other = odr.CubicSpline({1.0: odr.Poly3.from_odr(1.0, 2.0, 0.0, 0.0, 0.0)})
    assert spline.size() == 1
    assert not spline.empty()
    assert spline.get(0.5) == pytest.approx(1.0)
    assert spline.add(other).get(1.0) == pytest.approx(3.0)
    assert spline.negate().get(0.0) == pytest.approx(-1.0)
    assert len(spline.approximate_linear(0.5, 0.0, 2.0)) >= 2

    line = odr.Line(0.0, 0.0, 0.0, 0.0, 10.0)
    arc = odr.Arc(0.0, 0.0, 0.0, 0.0, 10.0, 0.1)
    spiral = odr.Spiral(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.1)
    param = odr.ParamPoly3(
        0.0,
        0.0,
        0.0,
        0.0,
        10.0,
        0.0,
        10.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    for geom in (line, arc, spiral, param):
        assert_vec_finite(geom.get_xy(5.0))
        assert_vec_finite(geom.get_grad(5.0))
        assert len(geom.approximate_linear(1.0)) >= 2

    ref_line = odr.RefLine("r", 10.0)
    ref_line.s0_to_geometry[0.0] = line
    ref_line.elevation_profile.s0_to_poly[0.0] = odr.Poly3.from_odr(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    assert ref_line.get_geometry_s0(5.0) == 0.0
    assert ref_line.get_geometry(5.0) is line
    assert len(ref_line.get_line(0.0, 10.0, 1.0)) >= 2
    assert 0.0 <= ref_line.match(5.0, 0.0) <= 10.0


def test_mesh_obj_and_routing_unreachable() -> None:
    mesh = odr.Mesh3D(vertices=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
    mesh.indices.extend([0, 1, 2])
    mesh.normals.append((0.0, 0.0, 1.0))
    obj_text = mesh.get_obj()
    assert "v 0.0 0.0 0.0" in obj_text
    assert "f 1 2 3" in obj_text

    other = odr.Mesh3D(vertices=[(0.0, 0.0, 1.0)])
    mesh.add_mesh(other)
    assert len(mesh.vertices) == 4

    graph = odr.RoutingGraph()
    a = odr.LaneKey("a", 0.0, -1)
    b = odr.LaneKey("b", 0.0, -1)
    assert graph.shortest_path(a, b) == []
    assert a.to_string() == "a/0.000000/-1"


def test_real_chatt_file_smoke() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    xodr = repo_root / "datasets/chatt.xodr"
    odr_map = odr.load(xodr)

    assert len(odr_map.get_roads()) == 189
    assert len(odr_map.get_junctions()) == 24
    first = odr_map.get_roads()[0]
    assert len(first.get_lanesections()) == 1
    assert_vec_finite(first.ref_line.get_xyz(0.0))

    graph = odr_map.get_routing_graph()
    assert len(graph.edges) == 474


@pytest.mark.skipif(
    shutil.which("netconvert") is None,
    reason="SUMO netconvert is required for SUMO conversion tests",
)
def test_chatt_xodr_converts_to_sumo_net_and_back(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    xodr = repo_root / "datasets/chatt.xodr"
    net_file = tmp_path / "chatt.net.xml"
    roundtrip_xodr = tmp_path / "chatt_roundtrip.xodr"

    sumo_net = odr.opendrive_to_sumo_net(
        xodr,
        net_file=net_file,
        netconvert_options={"no-turnarounds": True},
    )

    assert type(sumo_net).__module__.startswith("sumolib.net")
    assert net_file.exists()
    assert len(sumo_net.getNodes()) > 0
    assert len(sumo_net.getEdges()) > 0
    assert sumo_net.getBoundary()[2] > sumo_net.getBoundary()[0]
    edge_280 = sumo_net.getEdge("-280")
    assert edge_280.getParam("pyopendrive.original_link_id") == "280"
    assert edge_280.getLanes()[0].getParam("pyopendrive.original_lane_id") == "-1"
    assert edge_280.getLanes()[1].getParam("pyopendrive.original_lane_id") == "-2"
    assert sumo_net.getNode("1").getParam("pyopendrive.original_node_id") == "1"

    roundtrip_map = odr.sumo_net_to_opendrive_map(
        sumo_net,
        xodr_file=roundtrip_xodr,
    )

    assert isinstance(roundtrip_map, odr.OpenDriveMap)
    assert roundtrip_xodr.exists()
    assert len(roundtrip_map.get_roads()) > 0
    assert len(roundtrip_map.get_junctions()) > 0
    assert roundtrip_map.get_road("280").id == "280"
    assert roundtrip_map.get_road("280").get_lanesections()[0].get_lane(-1).id == -1
    assert roundtrip_map.get_road("280").get_lanesections()[0].get_lane(-2).id == -2
    assert_vec_finite(roundtrip_map.get_roads()[0].ref_line.get_xyz(0.0))


@pytest.mark.skipif(
    shutil.which("netconvert") is None,
    reason="SUMO netconvert is required for SUMO conversion tests",
)
def test_chatt_sumo_net_xml_converts_to_opendrive_map(tmp_path: Path) -> None:
    sumolib_net = pytest.importorskip("sumolib.net")
    repo_root = Path(__file__).resolve().parents[1]
    net_file = repo_root / "datasets/chatt.net.xml"
    roundtrip_xodr = tmp_path / "chatt_from_sumo.xodr"

    sumo_net = sumolib_net.readNet(str(net_file), withInternal=True)

    assert len(sumo_net.getNodes()) > 0
    assert len(sumo_net.getEdges()) > 0

    odr_map = odr.sumo_net_to_opendrive_map(
        sumo_net,
        net_file=net_file,
        xodr_file=roundtrip_xodr,
    )

    assert isinstance(odr_map, odr.OpenDriveMap)
    assert roundtrip_xodr.exists()
    assert len(odr_map.get_roads()) > 0
    assert len(odr_map.get_junctions()) > 0
    source_edge_ids = [
        edge.get("id")
        for edge in ET.parse(net_file).getroot().findall("edge")
        if edge.get("id") and edge.get("function") != "internal"
    ]
    roundtrip_road_ids = [
        road.get("id")
        for road in ET.parse(roundtrip_xodr).getroot().findall("road")
        if road.get("junction", "-1") == "-1"
    ]
    assert roundtrip_road_ids[: len(source_edge_ids)] == source_edge_ids
    assert_vec_finite(odr_map.get_roads()[0].ref_line.get_xyz(0.0))


def test_xodr_from_net_xml_restores_road_references_to_planview_roads(
    tmp_path: Path,
) -> None:
    import pyopendrive.__xodr_sumo as xodr_sumo

    net_file = tmp_path / "network.net.xml"
    net_file.write_text(
        """<net>
        <edge id="edge_a"/>
        <edge id="edge_b"/>
        </net>""",
        encoding="utf-8",
    )
    xodr_file = tmp_path / "network.xodr"
    xodr_file.write_text(
        """<OpenDRIVE>
        <road id="100" length="10" junction="-1">
            <link><successor elementType="road" elementId="101" contactPoint="start"/></link>
            <planView>
                <geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry>
            </planView>
        </road>
        <road id="101" length="10" junction="-1">
            <link><predecessor elementType="road" elementId="100" contactPoint="end"/></link>
            <planView>
                <geometry s="0" x="10" y="0" hdg="0" length="10"><line/></geometry>
            </planView>
        </road>
        <junction id="1">
            <connection id="0" incomingRoad="100" connectingRoad="101" contactPoint="start"/>
            <priority high="100" low="101"/>
        </junction>
        </OpenDRIVE>""",
        encoding="utf-8",
    )

    xodr_sumo._restore_opendrive_ids(xodr_file, net_file)

    root = ET.parse(xodr_file).getroot()
    roads = {road.get("id"): road for road in root.findall("road")}
    assert set(roads) == {"edge_a", "edge_b"}
    assert roads["edge_a"].find("./link/successor").get("elementId") == "edge_b"
    assert roads["edge_b"].find("./link/predecessor").get("elementId") == "edge_a"
    assert roads["edge_a"].find("./planView/geometry") is not None
    assert roads["edge_b"].find("./planView/geometry") is not None

    connection = root.find("./junction/connection")
    assert connection.get("incomingRoad") == "edge_a"
    assert connection.get("connectingRoad") == "edge_b"
    priority = root.find("./junction/priority")
    assert priority.get("high") == "edge_a"
    assert priority.get("low") == "edge_b"


def test_bad_lane_section_without_center_lane_raises(tmp_path: Path) -> None:
    xodr = tmp_path / "bad.xodr"
    xodr.write_text(
        """<OpenDRIVE><road id="1" length="1" junction="-1"><planView>
        <geometry s="0" x="0" y="0" hdg="0" length="1"><line/></geometry>
        </planView><lanes><laneSection s="0"><right><lane id="-1">
        <width sOffset="0" a="1" b="0" c="0" d="0"/></lane></right>
        </laneSection></lanes></road></OpenDRIVE>""",
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="lane section does not have lane #0"):
        odr.OpenDriveMap(str(xodr))
