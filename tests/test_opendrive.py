"""Tests for trace_fixer.export.opendrive: XML emission from the road
geometry plan (export.road_geometry) -- arc/line plan-view geometry,
multiple laneSections, and static objects.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"
SAMPLE2_DIR = REPO_ROOT / "data" / "traces" / "sample2"


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")


@pytest.fixture()
def sample2():
    from trace_fixer.scene import load_trace

    return load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")


def test_generated_opendrive_is_well_formed_xml(sample1):
    from trace_fixer.export.opendrive import generate_opendrive

    root = ET.fromstring(generate_opendrive(sample1))
    assert root.tag == "OpenDRIVE"
    assert root.find("road") is not None


def test_plan_view_uses_arcs_not_only_straight_lines(sample1):
    """sample1 has real curvature (it's not a perfectly straight road) --
    confirm the generator actually emits <arc> segments, not just <line>,
    now that it fits real heading instead of chord direction."""
    from trace_fixer.export.opendrive import generate_opendrive

    root = ET.fromstring(generate_opendrive(sample1))
    geometries = root.findall("./road/planView/geometry")
    assert len(geometries) > 10
    arc_count = sum(1 for g in geometries if g.find("arc") is not None)
    line_count = sum(1 for g in geometries if g.find("line") is not None)
    assert arc_count + line_count == len(geometries)
    assert arc_count > 0  # a real driven road has *some* curvature somewhere


def test_plan_view_geometry_is_contiguous_in_s_and_heading(sample1):
    """Each geometry's start s must equal the previous one's s + length (no
    gaps/overlaps), and each arc/line's *implied end heading* (start hdg +
    curvature * length, 0 curvature for a line) must match the next
    segment's declared start hdg -- otherwise the road has a visible kink.
    Raw heading is *not* expected to barely change between points -- a real
    curve genuinely changes heading from one sample to the next.
    """
    import math

    from trace_fixer.export.opendrive import generate_opendrive

    root = ET.fromstring(generate_opendrive(sample1))
    geometries = root.findall("./road/planView/geometry")
    prev_end_s = 0.0
    prev_implied_end_hdg = None
    for g in geometries:
        s = float(g.get("s"))
        length = float(g.get("length"))
        hdg = float(g.get("hdg"))
        assert s == pytest.approx(prev_end_s, abs=1e-2)
        if prev_implied_end_hdg is not None:
            diff = ((hdg - prev_implied_end_hdg + math.pi) % (2 * math.pi)) - math.pi
            assert abs(diff) < 1e-3
        arc = g.find("arc")
        curvature = float(arc.get("curvature")) if arc is not None else 0.0
        prev_end_s = s + length
        prev_implied_end_hdg = hdg + curvature * length
    road_length = float(root.find("road").get("length"))
    assert prev_end_s == pytest.approx(road_length, abs=1e-2)


def test_lanes_emits_one_or_more_sections_matching_the_plan(sample1, sample2):
    from trace_fixer.export.opendrive import generate_opendrive
    from trace_fixer.export.road_geometry import build_road_geometry_plan

    for trace in (sample1, sample2):
        plan = build_road_geometry_plan(trace)
        root = ET.fromstring(generate_opendrive(trace))
        sections = root.findall("./road/lanes/laneSection")
        assert len(sections) == len(plan.lane_sections)
        for xml_section, plan_section in zip(sections, plan.lane_sections):
            assert float(xml_section.get("s")) == pytest.approx(plan_section.s_start, abs=1e-2)
            right_lanes = xml_section.findall("./right/lane")
            assert len(right_lanes) == plan_section.num_lanes
            for lane, width in zip(right_lanes, plan_section.lane_widths_m):
                w = lane.find("width")
                assert float(w.get("a")) == pytest.approx(width, abs=1e-2)


def test_objects_emitted_for_every_static_object(sample1):
    from trace_fixer.export.opendrive import generate_opendrive

    root = ET.fromstring(generate_opendrive(sample1))
    objects = root.findall("./road/objects/object")
    assert len(objects) == len(sample1.annotation.static_objects)
    names = {o.get("name") for o in objects}
    assert "Traffic Sign" in names
    for o in objects:
        assert o.get("type")  # every object has *some* type
        assert float(o.get("s")) >= 0


def test_no_objects_element_when_trace_has_no_static_objects(sample1):
    """A trace with no static_objects shouldn't emit an empty <objects/>
    at all -- keeps the file minimal when there's nothing to say."""
    import copy

    from trace_fixer.export.opendrive import generate_opendrive

    trace = copy.deepcopy(sample1)
    trace.annotation.static_objects.clear()
    root = ET.fromstring(generate_opendrive(trace))
    assert root.find("./road/objects") is None


def test_lane_offset_is_emitted_per_section_and_is_non_zero(sample1):
    """Regression guard for the "exported cars sit off the exported road"
    bug: the reference line is the ego's driven path, which runs down the
    middle of its lane, so the lane stack has to be shifted by roughly half
    a lane via <laneOffset>. Hardcoding a=0 (what this used to emit)
    anchored the whole carriageway half a lane too far right.
    """
    from trace_fixer.export.opendrive import generate_opendrive
    from trace_fixer.export.road_geometry import build_road_geometry_plan

    plan = build_road_geometry_plan(sample1)
    root = ET.fromstring(generate_opendrive(sample1))
    offsets = root.findall("./road/lanes/laneOffset")
    assert len(offsets) == len(plan.lane_sections)
    assert any(abs(float(o.get("a"))) > 0.5 for o in offsets), "lane stack is still anchored on the reference line"
    for offset, section in zip(offsets, plan.lane_sections):
        assert float(offset.get("a")) == pytest.approx(section.center_offset_m, abs=1e-3)
        assert float(offset.get("s")) == pytest.approx(section.s_start, abs=1e-3)


def test_lane_offsets_precede_lane_sections(sample1):
    """ASAM OpenDRIVE fixes the order inside <lanes>: every laneOffset
    before any laneSection."""
    from trace_fixer.export.opendrive import generate_opendrive

    root = ET.fromstring(generate_opendrive(sample1))
    tags = [child.tag for child in root.find("./road/lanes")]
    assert tags, "no lane records emitted"
    assert tags == sorted(tags, key=lambda t: 0 if t == "laneOffset" else 1)


def test_left_lanes_are_emitted_with_positive_ids_in_spec_order(sample1):
    """Vehicles left of the ego need road surface under them; without left
    lanes they're off-road by construction. When a section has them, they
    must be positive-id and precede <center>/<right> per the spec."""
    from trace_fixer.export.opendrive import generate_opendrive
    from trace_fixer.export.road_geometry import build_road_geometry_plan

    plan = build_road_geometry_plan(sample1)
    root = ET.fromstring(generate_opendrive(sample1))
    sections = root.findall("./road/lanes/laneSection")
    assert len(sections) == len(plan.lane_sections)

    for section_el, plan_section in zip(sections, plan.lane_sections):
        order = [c.tag for c in section_el]
        assert order == sorted(order, key=lambda t: {"left": 0, "center": 1, "right": 2}[t])
        left_lanes = section_el.findall("./left/lane")
        assert len(left_lanes) == len(plan_section.left_lane_widths_m)
        for lane in left_lanes:
            assert int(lane.get("id")) > 0
        # emitted outermost-first, so ids descend
        ids = [int(lane.get("id")) for lane in left_lanes]
        assert ids == sorted(ids, reverse=True)
