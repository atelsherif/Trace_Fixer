"""Human/machine-readable "problem report" export: a standalone list of
everything validation flagged (category, severity, vehicle, time range,
description, whether the fix engine resolved it), separate from the
annotation/ADMA files themselves -- meant for an annotation QA team to
triage without needing to open the XML.
"""
from __future__ import annotations

from datetime import datetime, timezone
from xml.dom import minidom
from xml.etree.ElementTree import Element, SubElement, tostring

from trace_fixer.models import Trace

SEVERITY_ORDER = {"high": 0, "medium": 1, "low": 2}


def _sorted_issues(trace: Trace):
    return sorted(trace.issues, key=lambda i: (i.t_start_us, SEVERITY_ORDER.get(i.severity, 9)))


def _summary_counts(trace: Trace) -> dict:
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
    summary = _summary_counts(trace)

    lines = [
        "Trace Fixer -- Problem Report",
        f"Trace:          {trace.trace_id}",
        f"Generated:      {now}",
        f"Sync offset:    {trace.sync_offset_us / 1000:.1f} ms",
        "",
        f"Summary: {summary['total']} issue(s) "
        f"({summary['high']} high, {summary['medium']} medium, {summary['low']} low; "
        f"{summary['fixed']} fixed, {summary['unfixed']} open)",
    ]
    for category, n in sorted(summary["by_category"].items()):
        lines.append(f"  {category}: {n}")
    lines.append("")

    if not trace.issues:
        lines.append("No issues found.")
        return "\n".join(lines) + "\n"

    lines.append("Issues:")
    lines.append("-" * 70)
    for issue in _sorted_issues(trace):
        who = f"Vehicle {issue.vehicle_id}" if issue.vehicle_id is not None else "Ego (ADMA)"
        t_start_s = (issue.t_start_us - t0_us) / 1e6
        t_end_s = (issue.t_end_us - t0_us) / 1e6
        lines.append(
            f"[{issue.severity.upper()}] {issue.category.upper()} -- {who} "
            f"(t={t_start_s:.2f}s - {t_end_s:.2f}s)"
        )
        lines.append(f"  {issue.description}")
        lines.append(f"  Fixed: {'yes' if issue.fixed else 'no'}")
        lines.append("")

    return "\n".join(lines) + "\n"


def generate_xml_report(trace: Trace) -> str:
    t0_us = trace.ego.t0_us
    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    summary = _summary_counts(trace)

    root = Element(
        "TraceProblemReport",
        {
            "traceId": trace.trace_id,
            "generatedAt": now,
            "syncOffsetUs": str(trace.sync_offset_us),
        },
    )
    SubElement(
        root,
        "Summary",
        {
            "total": str(summary["total"]),
            "high": str(summary["high"]),
            "medium": str(summary["medium"]),
            "low": str(summary["low"]),
            "fixed": str(summary["fixed"]),
            "unfixed": str(summary["unfixed"]),
        },
    )
    issues_el = SubElement(root, "Issues")
    for issue in _sorted_issues(trace):
        issue_el = SubElement(
            issues_el,
            "Issue",
            {
                "id": issue.issue_id,
                "category": issue.category,
                "severity": issue.severity,
                "vehicleId": str(issue.vehicle_id) if issue.vehicle_id is not None else "",
                "tStartSeconds": f"{(issue.t_start_us - t0_us) / 1e6:.3f}",
                "tEndSeconds": f"{(issue.t_end_us - t0_us) / 1e6:.3f}",
                "fixable": "true" if issue.fixable else "false",
                "fixed": "true" if issue.fixed else "false",
            },
        )
        desc_el = SubElement(issue_el, "Description")
        desc_el.text = issue.description

    xml_bytes = tostring(root, encoding="utf-8")
    return minidom.parseString(xml_bytes).toprettyxml(indent="  ")
