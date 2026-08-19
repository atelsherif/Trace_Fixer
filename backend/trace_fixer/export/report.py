"""Trace summary export (txt / xml): scene composition (moving-object and
static-object counts by type), ego braking events, overtake events, and
data-quality issues found by validation -- a standalone report meant for a
QA/review team to triage a trace without opening the annotation XML or
replaying it in the GUI.
"""
from __future__ import annotations

from datetime import datetime, timezone
from xml.dom import minidom
from xml.etree.ElementTree import Element, SubElement, tostring

from trace_fixer.analysis import TraceSummary, build_trace_summary
from trace_fixer.models import Trace

SEVERITY_ORDER = {"high": 0, "medium": 1, "low": 2}

_OVERTAKE_LABELS = {
    "vehicle_overtakes_ego": "Vehicle overtakes ego",
    "ego_overtakes_vehicle": "Ego overtakes vehicle",
}


def _sorted_issues(trace: Trace):
    return sorted(trace.issues, key=lambda i: (i.t_start_us, SEVERITY_ORDER.get(i.severity, 9)))


def _issue_counts(trace: Trace) -> dict:
    counts = {"total": len(trace.issues), "high": 0, "medium": 0, "low": 0, "fixed": 0, "unfixed": 0}
    by_category: dict[str, int] = {}
    for issue in trace.issues:
        counts[issue.severity] = counts.get(issue.severity, 0) + 1
        counts["fixed" if issue.fixed else "unfixed"] += 1
        by_category[issue.category] = by_category.get(issue.category, 0) + 1
    counts["by_category"] = by_category
    return counts


def generate_txt_report(trace: Trace) -> str:
    t0_us = trace.ego.t0_us
    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    duration_s = (trace.ego.t1_us - t0_us) / 1e6
    summary = build_trace_summary(trace)
    issues = _issue_counts(trace)

    def rel_s(t_us: int) -> float:
        return (t_us - t0_us) / 1e6

    lines = [
        "Trace Summary Report",
        f"Trace:          {trace.trace_id}",
        f"Generated:      {now}",
        f"Duration:       {duration_s:.2f} s",
        f"Sync offset:    {trace.sync_offset_us / 1000:.1f} ms",
        "",
        f"Scene composition: {sum(summary.object_counts.values())} object(s)",
    ]
    for obj_type, n in sorted(summary.object_counts.items()):
        lines.append(f"  {obj_type}: {n}")
    lines.append("")

    lines.append(f"Static objects: {sum(summary.static_object_counts.values())} object(s)")
    for obj_type, n in sorted(summary.static_object_counts.items()):
        lines.append(f"  {obj_type}: {n}")
    lines.append("")

    lines.append(f"Ego braking events: {len(summary.braking_events)}")
    if not summary.braking_events:
        lines.append("  None detected.")
    for ev in summary.braking_events:
        lines.append(
            f"  [{ev.severity.upper()}] t={rel_s(ev.t_start_us):.2f}s-{rel_s(ev.t_end_us):.2f}s, "
            f"peak {ev.peak_decel_mps2:.1f} m/s^2"
        )
    lines.append("")

    lines.append(f"Overtake events: {len(summary.overtake_events)}")
    if not summary.overtake_events:
        lines.append("  None detected.")
    for ev in summary.overtake_events:
        label = _OVERTAKE_LABELS[ev.direction]
        lines.append(
            f"  t={rel_s(ev.t_us):.2f}s: {label} (vehicle {ev.vehicle_id}, "
            f"lateral offset {ev.lateral_offset_m:.1f} m)"
        )
    lines.append("")

    lines.append(
        f"Data-quality issues: {issues['total']} "
        f"({issues['high']} high, {issues['medium']} medium, {issues['low']} low; "
        f"{issues['fixed']} fixed, {issues['unfixed']} open)"
    )
    for category, n in sorted(issues["by_category"].items()):
        lines.append(f"  {category}: {n}")
    lines.append("")

    if not trace.issues:
        lines.append("No data-quality issues found.")
        return "\n".join(lines) + "\n"

    lines.append("Issue detail:")
    lines.append("-" * 70)
    for issue in _sorted_issues(trace):
        who = f"Vehicle {issue.vehicle_id}" if issue.vehicle_id is not None else "Ego (ADMA)"
        lines.append(
            f"[{issue.severity.upper()}] {issue.category.upper()} -- {who} "
            f"(t={rel_s(issue.t_start_us):.2f}s - {rel_s(issue.t_end_us):.2f}s)"
        )
        lines.append(f"  {issue.description}")
        lines.append(f"  Fixed: {'yes' if issue.fixed else 'no'}")
        lines.append("")

    return "\n".join(lines) + "\n"


def generate_xml_report(trace: Trace) -> str:
    t0_us = trace.ego.t0_us
    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    duration_s = (trace.ego.t1_us - t0_us) / 1e6
    summary = build_trace_summary(trace)
    issues = _issue_counts(trace)

    def rel_s(t_us: int) -> float:
        return (t_us - t0_us) / 1e6

    root = Element(
        "TraceSummary",
        {
            "traceId": trace.trace_id,
            "generatedAt": now,
            "durationSeconds": f"{duration_s:.3f}",
            "syncOffsetUs": str(trace.sync_offset_us),
        },
    )

    scene_el = SubElement(root, "SceneComposition", {"totalObjects": str(sum(summary.object_counts.values()))})
    for obj_type, n in sorted(summary.object_counts.items()):
        SubElement(scene_el, "ObjectType", {"type": obj_type, "count": str(n)})

    static_el = SubElement(root, "StaticObjects", {"totalObjects": str(sum(summary.static_object_counts.values()))})
    for obj_type, n in sorted(summary.static_object_counts.items()):
        SubElement(static_el, "ObjectType", {"type": obj_type, "count": str(n)})

    braking_el = SubElement(root, "BrakingEvents", {"count": str(len(summary.braking_events))})
    for ev in summary.braking_events:
        SubElement(
            braking_el,
            "BrakingEvent",
            {
                "severity": ev.severity,
                "tStartSeconds": f"{rel_s(ev.t_start_us):.3f}",
                "tEndSeconds": f"{rel_s(ev.t_end_us):.3f}",
                "peakDecelMps2": f"{ev.peak_decel_mps2:.2f}",
            },
        )

    overtake_el = SubElement(root, "OvertakeEvents", {"count": str(len(summary.overtake_events))})
    for ev in summary.overtake_events:
        SubElement(
            overtake_el,
            "OvertakeEvent",
            {
                "vehicleId": str(ev.vehicle_id),
                "tSeconds": f"{rel_s(ev.t_us):.3f}",
                "direction": ev.direction,
                "lateralOffsetMeters": f"{ev.lateral_offset_m:.2f}",
            },
        )

    issues_el = SubElement(
        root,
        "DataQualityIssues",
        {
            "total": str(issues["total"]),
            "high": str(issues["high"]),
            "medium": str(issues["medium"]),
            "low": str(issues["low"]),
            "fixed": str(issues["fixed"]),
            "unfixed": str(issues["unfixed"]),
        },
    )
    for issue in _sorted_issues(trace):
        issue_el = SubElement(
            issues_el,
            "Issue",
            {
                "id": issue.issue_id,
                "category": issue.category,
                "severity": issue.severity,
                "vehicleId": str(issue.vehicle_id) if issue.vehicle_id is not None else "",
                "tStartSeconds": f"{rel_s(issue.t_start_us):.3f}",
                "tEndSeconds": f"{rel_s(issue.t_end_us):.3f}",
                "fixable": "true" if issue.fixable else "false",
                "fixed": "true" if issue.fixed else "false",
            },
        )
        desc_el = SubElement(issue_el, "Description")
        desc_el.text = issue.description

    xml_bytes = tostring(root, encoding="utf-8")
    return minidom.parseString(xml_bytes).toprettyxml(indent="  ")
