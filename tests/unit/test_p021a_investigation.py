"""P-021A investigation tooling tests. Recovery acceptance는 여기서 바꾸지 않는다."""

from controller.p021a_investigation import (
    SNAPSHOT_VARIABLES,
    classify_trials,
    command_observation,
    confirm_root_cause,
    motion_observation,
    parse_snapshot,
    snapshot_diff,
)


def _snapshot(**overrides: str) -> dict[str, str]:
    base = {name: "0" for name in SNAPSHOT_VARIABLES}
    base.update(
        {
            "Motor[1].Ctrl": "Sys.ServoCtrl",
            "Motor[1].Status[0]": "$7800",
            "Motor[1].Status[1]": "$0",
            "Motor[1].ClosedLoop": "1",
            "Motor[1].ServoCtrl": "1",
            "Motor[1].JogSpeed": "32",
            "Motor[1].MaxSpeed": "32",
            "Motor[1].JogTa": "-10",
            "Motor[1].JogTs": "-50",
            "Motor[1].AmpEna": "1",
        }
    )
    base.update(overrides)
    return base


def test_parser_keeps_raw_values_and_marks_missing_names() -> None:
    stdout = "\n".join(
        [
            "STDIN Open for ASCII Input",
            "Motor[1].JogSpeed=32",
            "\x06",
            "Motor[1].Ctrl=Sys.ServoCtrl",
            "\x06",
        ]
    )
    parsed = parse_snapshot(stdout, ("Motor[1].JogSpeed", "Motor[1].Ctrl", "Motor[1].ProgJogPos"))
    assert parsed["Motor[1].JogSpeed"] == "32"
    assert parsed["Motor[1].Ctrl"] == "Sys.ServoCtrl"
    assert parsed["Motor[1].ProgJogPos"] == "NOT_AVAILABLE"


def test_snapshot_diff_labels_motion_reconnect_and_missing() -> None:
    before = _snapshot(**{"Motor[1].Pos": "0", "Motor[1].DesPos": "0", "Motor[1].ActPos": "0"})
    after = _snapshot(**{"Motor[1].Pos": "1", "Motor[1].DesPos": "1", "Motor[1].ActPos": "1"})
    reconnect_before = _snapshot(
        **{"Motor[1].Pos": "1", "Motor[1].DesPos": "1", "Motor[1].ActPos": "1", "Motor[1].ClosedLoop": "0"}
    )
    reconnect_after = dict(reconnect_before)
    reconnect_before["Motor[1].ProgJogPos"] = "NOT_AVAILABLE"
    rows = {row["variable"]: row for row in snapshot_diff(before, after, reconnect_before, reconnect_after)}
    assert rows["Motor[1].JogSpeed"]["tags"] == ["UNCHANGED"]
    assert "CHANGED_ON_MOTION" in rows["Motor[1].Pos"]["tags"]
    assert "CHANGED_AFTER_RECONNECT" in rows["Motor[1].ClosedLoop"]["tags"]
    assert rows["Motor[1].ProgJogPos"]["tags"] == ["NOT_AVAILABLE"]


def test_ack_does_not_make_an_unchanged_position_successful() -> None:
    before = _snapshot(**{"Motor[1].Pos": "0", "Motor[1].ActPos": "0", "Motor[1].DesPos": "0"})
    after = dict(before)
    observed = motion_observation(before, after, 1)
    response = command_observation("STDIN\n\x06\n// *** exit\n", "", 1)
    assert response["ack_present"] is True
    assert response["error_present"] is False
    assert response["ack_is_motion_success"] is False
    assert observed["reached"] is False
    assert observed["despos_changed"] is False


def test_fresh_failure_is_baseline_not_reconnect() -> None:
    trials = [
        {"fresh_motion_reached": False, "reconnect_ran": False, "explanatory_drift": []},
        {"fresh_motion_reached": False, "reconnect_ran": False, "explanatory_drift": []},
    ]
    result = classify_trials(trials)
    assert result["classification"] == "RC-3"
    assert result["fresh_baseline_reproducible"] is False
    assert result["root_cause_confirmed"] is False


def test_reconnect_failure_with_and_without_drift() -> None:
    drifted = classify_trials(
        [
            {
                "fresh_motion_reached": True,
                "reconnect_ran": True,
                "reconnect_motion_reached": False,
                "explanatory_drift": ["Motor[1].ClosedLoop"],
            }
        ]
    )
    quiet = classify_trials(
        [
            {
                "fresh_motion_reached": True,
                "reconnect_ran": True,
                "reconnect_motion_reached": False,
                "explanatory_drift": [],
            }
        ]
    )
    assert drifted["classification"] == "RC-1"
    assert quiet["classification"] == "RC-2"
    assert confirm_root_cause(True, False) is False
    assert drifted["root_cause_confirmed"] is False
    assert quiet["classification"] != "RC-5"


def test_mixed_results_are_intermittent_and_correlation_is_not_confirmation() -> None:
    result = classify_trials(
        [
            {"fresh_motion_reached": True, "reconnect_ran": True, "reconnect_motion_reached": True, "explanatory_drift": []},
            {"fresh_motion_reached": False, "reconnect_ran": False, "explanatory_drift": []},
        ]
    )
    assert result["classification"] == "RC-4"
    assert confirm_root_cause(True, True) is True
    assert result["root_cause_confirmed"] is False
