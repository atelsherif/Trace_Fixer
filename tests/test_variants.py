"""Tests for trace_fixer.variants: preset and randomized "alternative
scenario" generation -- perturbing one selected surrounding vehicle's
recorded trajectory while leaving the ego and the road untouched.
"""
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation

    trace = load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")
    run_validation(trace)
    return trace


def test_preset_variants_are_generated_and_named(sample1):
    from trace_fixer.variants import PRESET_SPECS, generate_preset_variants

    variants = generate_preset_variants(sample1, count=5)
    assert len(variants) == 5
    kinds = {v.kind for v in variants}
    preset_kinds = {s.kind for s in PRESET_SPECS}
    # sample1 has a cut-in, a lead vehicle, and an overtake -- every preset applies
    assert kinds == preset_kinds
    for v in variants:
        assert v.name
        assert v.description
        assert v.vehicle_id in sample1.annotation.vehicles


def test_variant_traces_are_independent_deep_copies(sample1):
    """Perturbing a variant must never mutate the source trace."""
    from trace_fixer.variants import generate_preset_variants

    original_positions = {
        vid: [(o.x_rel, o.y_rel) for o in track.observations]
        for vid, track in sample1.annotation.vehicles.items()
    }
    variants = generate_preset_variants(sample1, count=5)
    assert variants  # sanity
    for vid, track in sample1.annotation.vehicles.items():
        assert [(o.x_rel, o.y_rel) for o in track.observations] == original_positions[vid]

    # at least one variant actually changed the picked vehicle's positions
    changed = False
    for v in variants:
        vt = v.trace.annotation.vehicles[v.vehicle_id]
        if [(o.x_rel, o.y_rel) for o in vt.observations] != original_positions[v.vehicle_id]:
            changed = True
    assert changed


def test_variant_trace_id_is_derived_and_unique(sample1):
    from trace_fixer.variants import generate_preset_variants

    variants = generate_preset_variants(sample1, count=5)
    trace_ids = [v.trace.trace_id for v in variants]
    assert len(set(trace_ids)) == len(trace_ids)
    for tid in trace_ids:
        assert tid.startswith("sample1__")


def test_variant_scene_and_exports_work_unmodified(sample1):
    """Every existing export path must work on a variant's Trace with zero
    changes -- that's the whole point of representing a variant as a
    plain Trace object."""
    from trace_fixer.export.adp_yaml import generate_adp_scenario_yaml
    from trace_fixer.export.openscenario import generate_openscenario
    from trace_fixer.scene import build_scene_json
    from trace_fixer.variants import generate_preset_variants

    variant = generate_preset_variants(sample1, count=1)[0]
    scene = build_scene_json(variant.trace)
    assert len(scene["vehicles"]) == len(sample1.annotation.vehicles)

    yaml_text = generate_adp_scenario_yaml(variant.trace)
    assert variant.trace.trace_id in yaml_text

    xosc_text = generate_openscenario(variant.trace, f"{variant.trace.trace_id}.xodr")
    assert "OpenSCENARIO" in xosc_text


def test_perturbed_observations_are_marked_fixed_so_annotation_export_includes_them(sample1, tmp_path):
    """Regression guard: export.annotation_writer only patches observations
    flagged `fixed=True` -- if a variant's perturbation didn't set that
    flag, its Fixed Trace export would silently reproduce the original,
    unperturbed annotation XML."""
    from pathlib import Path

    from trace_fixer.export.annotation_writer import write_annotation_xml
    from trace_fixer.variants import generate_preset_variants

    variant = generate_preset_variants(sample1, count=1)[0]
    track = variant.trace.annotation.vehicles[variant.vehicle_id]
    assert all(o.fixed for o in track.observations)

    out_path = tmp_path / "out.xml"
    original_path = Path("data/traces/sample1/annotation.xml")
    write_annotation_xml(variant.trace, original_path, out_path, include_predictions=False)

    import xml.etree.ElementTree as ET

    written = ET.parse(out_path)
    for rv in written.getroot().find("vehicles").findall("rect_vehicle"):
        if int(rv.findtext("id")) != variant.vehicle_id:
            continue
        by_frame = {int(ts.attrib["frame"]): ts for ts in rv.find("timestamps").findall("rect_vehicle_timestamp")}
        for o in track.observations:
            ts_el = by_frame.get(o.frame)
            if ts_el is None:
                continue
            xp = float(ts_el.find("bounding_box/vehicle_bb/coordinates/xp").text)
            assert xp == pytest.approx(o.x_rel, abs=1e-6)


def test_hard_brake_lead_actually_reduces_the_gap(sample1):
    from trace_fixer.variants import PRESET_SPECS, _build_variant

    spec = next(s for s in PRESET_SPECS if s.kind == "hard_brake_lead")
    vehicle_id = spec.pick(sample1)
    assert vehicle_id is not None
    original = sorted(sample1.annotation.vehicles[vehicle_id].observations, key=lambda o: o.t_us)
    variant = _build_variant(sample1, vehicle_id, spec.apply, "test-id", spec.kind, spec.name, spec.description, "preset")
    perturbed = sorted(variant.trace.annotation.vehicles[vehicle_id].observations, key=lambda o: o.t_us)

    # the gap at the end of the windowed step must be smaller than originally recorded
    assert perturbed[-1].x_rel < original[-1].x_rel


def test_closer_following_never_produces_a_non_positive_gap(sample1):
    from trace_fixer.variants import PRESET_SPECS, _build_variant

    spec = next(s for s in PRESET_SPECS if s.kind == "closer_following")
    vehicle_id = spec.pick(sample1)
    assert vehicle_id is not None
    variant = _build_variant(sample1, vehicle_id, spec.apply, "test-id", spec.kind, spec.name, spec.description, "preset")
    for o in variant.trace.annotation.vehicles[vehicle_id].observations:
        if o.x_rel > 0:
            assert o.x_rel >= 1.0  # the floor in _scale_gap


def test_randomized_variants_are_reproducible_with_the_same_seed(sample1):
    from trace_fixer.variants import generate_randomized_variants

    a = generate_randomized_variants(sample1, count=4, seed=123)
    b = generate_randomized_variants(sample1, count=4, seed=123)
    assert [(v.kind, v.vehicle_id) for v in a] == [(v.kind, v.vehicle_id) for v in b]
    for va, vb in zip(a, b):
        pa = [(o.x_rel, o.y_rel) for o in va.trace.annotation.vehicles[va.vehicle_id].observations]
        pb = [(o.x_rel, o.y_rel) for o in vb.trace.annotation.vehicles[vb.vehicle_id].observations]
        assert pa == pb


def test_randomized_variants_differ_with_a_different_seed(sample1):
    from trace_fixer.variants import generate_randomized_variants

    a = generate_randomized_variants(sample1, count=4, seed=1)
    b = generate_randomized_variants(sample1, count=4, seed=2)
    assert [(v.kind, v.vehicle_id) for v in a] != [(v.kind, v.vehicle_id) for v in b]


def test_generate_randomized_variants_returns_empty_for_a_trace_with_no_eligible_vehicles():
    from trace_fixer.models import Annotation, EgoTrace, Trace
    from trace_fixer.variants import generate_randomized_variants

    empty_trace = Trace(trace_id="empty", ego=EgoTrace(poses=[]), annotation=Annotation(country_code=None))
    assert generate_randomized_variants(empty_trace, count=5) == []


def test_preset_variants_backfill_with_randomized_when_a_trace_lacks_some_signatures(sample1, monkeypatch):
    """If e.g. no cut-in exists in a trace, that preset slot is skipped and
    backfilled with a randomized variant so `count` is still honored."""
    import dataclasses

    import trace_fixer.variants as variants_mod

    patched_specs = [
        dataclasses.replace(spec, pick=(lambda trace: None)) if spec.kind in ("harder_cutin", "delayed_merge") else spec
        for spec in variants_mod.PRESET_SPECS
    ]
    monkeypatch.setattr(variants_mod, "PRESET_SPECS", patched_specs)
    result = variants_mod.generate_preset_variants(sample1, count=5)
    assert len(result) == 5
    assert not any(v.kind in ("harder_cutin", "delayed_merge") for v in result)
    assert any(v.source == "randomized" for v in result)
