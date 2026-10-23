"""단일 운영 화면. 색만으로 상태를 전하지 않는다."""

from __future__ import annotations

import html
import math

from hmi.model import HmiSnapshot
from hmi.view_model import CASES, format_deg_from_rad, format_mm, format_rad, format_seconds

_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>RoboTwin Control Lab</title>
<style>
body {{ margin: 0; font-family: "Segoe UI", sans-serif; background: #f4f6f8; color: #17202a; }}
header, footer {{ display: flex; justify-content: space-between; padding: 12px 20px; background: #102027; color: white; }}
main {{ padding: 16px; display: grid; gap: 12px; }}
section {{ background: white; border: 1px solid #cfd8dc; border-radius: 8px; padding: 12px 16px; }}
.top {{ display: grid; grid-template-columns: 320px 1fr; gap: 12px; }}
.lower {{ display: grid; grid-template-columns: 1.2fr 0.8fr; gap: 12px; }}
h1 {{ margin: 0; font-size: 20px; }}
h2 {{ margin: 0 0 8px; font-size: 14px; letter-spacing: 0.04em; }}
.decision {{ font-size: 28px; font-weight: 700; margin: 4px 0; }}
table {{ border-collapse: collapse; width: 100%; }}
th, td {{ text-align: left; padding: 4px 8px; border-bottom: 1px solid #eceff1; font-variant-numeric: tabular-nums; }}
button {{ margin: 0 8px 8px 0; padding: 8px 12px; }}
button:disabled {{ opacity: 0.45; }}
.note {{ color: #37474f; }}
svg {{ width: 100%; height: 220px; background: #fafafa; border: 1px solid #eceff1; }}
.project {{ font-weight: 700; }}
</style>
</head>
<body>
<header>
<h1>RoboTwin Control Lab</h1>
<div><span class="project">PROJECT TEST</span> · SYSTEM: {ui_state}</div>
</header>
<main>
<p class="note">Mode: {mode}. PROJECT TEST. Digital Twin Tracking Error. Not UR10e positioning accuracy. Not a safety certification.</p>
<div class="top">
<section>
<h2>COMMAND</h2>
<p>Case: {case_name}</p>
<p>Command target: {command_target}</p>
<form method="post" action="/select">{case_buttons}</form>
<form method="post" action="/validate"><button type="submit">VALIDATE</button></form>
<form method="post" action="/execute"><button type="submit" {execute_disabled}>EXECUTE</button></form>
<form method="post" action="/fault"><button type="submit" {execute_disabled}>INJECT TEST_FAULT</button></form>
<p data-execute-enabled="{execute_flag}">{execute_text}</p>
</section>
<section>
<h2>PRE-EXECUTION VALIDATION</h2>
<p class="decision" data-decision="{decision}">{decision}</p>
<p>{decision_meaning}</p>
<p>blocking_reason_codes: {reasons}</p>
<p>{collision}</p>
<p class="note">{summary}</p>
<ul>{checks}</ul>
</section>
</div>
<section>
<h2>DIGITAL TWIN — TARGET vs ACTUAL</h2>
{joints}
</section>
<div class="lower">
<section>
<h2>TRACKING ERROR</h2>
<p>Max joint error: {peak}</p>
<p>Completion time: {completion}</p>
<p>Settling time: {settling}</p>
{chart}
</section>
<section>
<h2>TCP</h2>
{tcp}
</section>
</div>
</main>
<footer>
<div>Controller: {controller_state}</div>
<div>Last Execution: {execution_state}</div>
<div data-settled="{settled}">Settled: {settled}</div>
</footer>
</body>
</html>
"""


def render(snapshot: HmiSnapshot) -> str:
    return _PAGE.format(
        ui_state=html.escape(snapshot.ui_state),
        mode=html.escape(snapshot.mode),
        case_name=html.escape(snapshot.case_name or "NOT RUN"),
        command_target=_command_target(snapshot),
        case_buttons=_case_buttons(snapshot.case_name),
        execute_disabled="" if snapshot.execute_enabled else "disabled",
        execute_flag="true" if snapshot.execute_enabled else "false",
        execute_text="Execute enabled" if snapshot.execute_enabled else "Execute disabled",
        decision=html.escape("FAULT" if snapshot.fault else snapshot.validation_decision),
        decision_meaning=_meaning(snapshot),
        reasons=html.escape(", ".join(snapshot.validation_reasons) if snapshot.validation_reasons else "none"),
        collision=_collision(snapshot),
        summary=html.escape(snapshot.validation_summary),
        checks=_check_list(snapshot),
        joints=_joints(snapshot),
        peak=_peak(snapshot),
        completion="NOT RUN" if snapshot.completion_time_s is None else html.escape(format_seconds(snapshot.completion_time_s)),
        settling="NOT RUN" if snapshot.settling_time_s is None else html.escape(format_seconds(snapshot.settling_time_s)),
        chart=_chart(snapshot),
        tcp=_tcp(snapshot),
        controller_state=html.escape(snapshot.controller_state or "NOT RUN"),
        execution_state=html.escape(snapshot.execution_state),
        settled=_settled(snapshot),
    )


def _meaning(snapshot: HmiSnapshot) -> str:
    if snapshot.ui_state == "FAULT" or snapshot.fault:
        return "FAULT. Prior validation was " + snapshot.validation_decision + ". Injected project fault. Execution did not finish and did not settle."
    if snapshot.validation_decision == "SAFE_TO_ATTEMPT":
        return "Pre-execution project checks passed."
    if snapshot.validation_decision == "REJECT":
        return "Command blocked before the controller."
    return "Validation has not been run."


def _collision(snapshot: HmiSnapshot) -> str:
    pair = snapshot.collision_pair or {}
    if not pair:
        return ""
    sample = snapshot.collision_sample or "not reported"
    left = html.escape(str(pair.get("a", "")))
    right = html.escape(str(pair.get("b", "")))
    return f"Collision: {left} ↔ {right}. First detected sample: {html.escape(sample)}."


def _check_list(snapshot: HmiSnapshot) -> str:
    if not snapshot.checks:
        return "<li>Checks: NOT RUN</li>"
    items = []
    for check in snapshot.checks:
        reason = check.get("reason_code") or ""
        items.append(
            "<li>"
            + html.escape(str(check.get("name")))
            + ": "
            + html.escape(str(check.get("result")))
            + (" " + html.escape(str(reason)) if reason else "")
            + "</li>"
        )
    return "".join(items)


def _command_target(snapshot: HmiSnapshot) -> str:
    if not snapshot.commanded_q:
        return "NOT RUN"
    parts = []
    for index, value in enumerate(snapshot.commanded_q, start=1):
        parts.append(f"J{index} {format_deg_from_rad(value)}")
    return html.escape(", ".join(parts))


def _joints(snapshot: HmiSnapshot) -> str:
    if not snapshot.tracking_started or snapshot.target_q is None or snapshot.actual_q is None or snapshot.joint_error is None:
        return "<p>Tracking not started</p>"
    latest = _joint_table("Latest sample", snapshot.joint_names, snapshot.target_q, snapshot.actual_q, snapshot.joint_error)
    if snapshot.peak_target_q is None or snapshot.peak_actual_q is None or snapshot.peak_joint_error is None:
        return latest
    peak = _joint_table(
        "Largest recorded difference",
        snapshot.joint_names,
        snapshot.peak_target_q,
        snapshot.peak_actual_q,
        snapshot.peak_joint_error,
    )
    return peak + latest


def _joint_table(title: str, names: list[str], target_q: list[float], actual_q: list[float], error: list[float]) -> str:
    rows = [f"<caption>{html.escape(title)}</caption>", "<tr><th>Joint</th><th>Target</th><th>Actual</th><th>Error</th></tr>"]
    for index, name in enumerate(names):
        rows.append(
            "<tr><td>"
            + html.escape(f"J{index + 1} {name}")
            + "</td><td>"
            + html.escape(_joint_value(target_q[index]))
            + "</td><td>"
            + html.escape(_joint_value(actual_q[index]))
            + "</td><td>"
            + html.escape(_joint_value(error[index]))
            + "</td></tr>"
        )
    return "<table>" + "".join(rows) + "</table>"


def _joint_value(value: object) -> str:
    shown = format_deg_from_rad(value)
    if shown == "INVALID":
        return "INVALID"
    return shown + " / " + format_rad(value)


def _tcp(snapshot: HmiSnapshot) -> str:
    if not snapshot.tracking_started or snapshot.target_tcp is None or snapshot.actual_tcp is None:
        return "<p>Tracking not started</p>"
    return (
        "<p>Digital Twin Tracking Error</p>"
        + "<p>Target TCP Position: "
        + _position(snapshot.target_tcp)
        + "</p><p>Actual TCP Position: "
        + _position(snapshot.actual_tcp)
        + "</p><p>Position Error: "
        + html.escape(format_mm(snapshot.tcp_position_error_m))
        + "</p><p>Orientation Error: "
        + html.escape(format_deg_from_rad(snapshot.tcp_orientation_error_rad))
        + "</p>"
    )


def _position(pose: dict) -> str:
    values = (pose.get("position_m") or [None, None, None])[:3]
    parts = [axis + " " + format_mm(value) for axis, value in zip(("x", "y", "z"), values, strict=True)]
    return html.escape(", ".join(parts))


def _peak(snapshot: HmiSnapshot) -> str:
    if snapshot.peak_joint_error_rad is None:
        return "NOT RUN"
    shown = format_deg_from_rad(snapshot.peak_joint_error_rad)
    if shown == "INVALID":
        return "INVALID"
    return html.escape(shown + " / " + format_rad(snapshot.peak_joint_error_rad))


def _settled(snapshot: HmiSnapshot) -> str:
    if snapshot.fault or snapshot.settled is False:
        return "NO"
    if snapshot.settled is True:
        return "YES"
    return "NOT RUN"


def _case_buttons(selected: str | None) -> str:
    buttons = []
    for name in CASES:
        pressed = "true" if name == selected else "false"
        buttons.append(
            f'<button type="submit" name="case" value="{html.escape(name)}" aria-pressed="{pressed}">{html.escape(name)}</button>'
        )
    return "".join(buttons)


def _chart(snapshot: HmiSnapshot) -> str:
    if not snapshot.series:
        return "<p>No tracking series</p>"
    if any(point["max_abs_joint_error_rad"] is None or not math.isfinite(float(point["time_s"])) for point in snapshot.series):
        return "<p>DATA ERROR</p>"
    width = 640
    height = 200
    left = 48
    right = 16
    top = 16
    bottom = 32
    times = [float(point["time_s"]) for point in snapshot.series]
    errors = [float(point["max_abs_joint_error_rad"]) for point in snapshot.series]
    t0 = min(times)
    t1 = max(times)
    e1 = max(errors) or 1.0
    span_t = t1 - t0 or 1.0

    def x_of(time_s: float) -> float:
        return left + (time_s - t0) / span_t * (width - left - right)

    def y_of(error: float) -> float:
        return top + (1.0 - error / e1) * (height - top - bottom)

    points = " ".join(f"{x_of(time_s):.2f},{y_of(error):.2f}" for time_s, error in zip(times, errors, strict=True))
    markers = []
    if snapshot.completed and snapshot.completion_time_s is not None and not snapshot.fault:
        markers.append(_marker(x_of(float(snapshot.completion_time_s)), "COMPLETED", "#1565c0", 28))
    if snapshot.settled is True and snapshot.settled_at_s is not None and not snapshot.fault:
        markers.append(_marker(x_of(float(snapshot.settled_at_s)), "SETTLED", "#2e7d32", 52))
    if snapshot.fault:
        markers.append(_marker(x_of(times[-1]), "FAULT", "#b71c1c", 28))
    return (
        f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="Max joint error versus time">'
        f'<text x="{left}" y="12">error (rad)</text>'
        f'<text x="{width - 70}" y="{height - 8}">time (s)</text>'
        f'<polyline fill="none" stroke="#263238" stroke-width="2" points="{points}"></polyline>'
        + "".join(markers)
        + "</svg>"
    )


def _marker(x_pos: float, label: str, color: str, text_y: int) -> str:
    return (
        f'<line x1="{x_pos:.2f}" y1="20" x2="{x_pos:.2f}" y2="160" stroke="{color}" stroke-dasharray="4 3"></line>'
        f'<text x="{x_pos + 4:.2f}" y="{text_y}" fill="{color}">{html.escape(label)}</text>'
    )
