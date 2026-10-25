"""P-021B pipeline classification tests."""

from controller.p021b_investigation import (
    NOT_MEASURED,
    causal_confirmation,
    classify_pipeline,
    classify_pmac_write,
    historical_matrix,
    jog_time_parameter_class,
    motion_completed,
)


def test_missing_historical_value_stays_unmeasured() -> None:
    matrix = historical_matrix({"P-019B_before_successful_jog": {"ServoCtrl": "1", "Status[0]": "$7800"}})
    assert matrix["ServoCtrl"]["P-019B_before_successful_jog"] == "1"
    assert matrix["JogTa"]["P-019B_before_successful_jog"] == NOT_MEASURED
    assert matrix["Coord"]["P-019B_before_successful_jog"] == NOT_MEASURED


def test_jogpos_change_without_despos_is_command_target_only() -> None:
    before = {"JogPos": "2", "DesPos": "0", "Pos": "0", "ActPos": "0", "DesVel": "0"}
    samples = [
        {"JogPos": "1", "DesPos": "0", "Pos": "0", "ActPos": "0", "DesVel": "0"},
        {"JogPos": "1", "DesPos": "0", "Pos": "0", "ActPos": "0", "DesVel": "0"},
    ]
    assert classify_pipeline(before, samples, "1") == "RB-1"


def test_one_success_is_not_causal_confirmation_and_ack_is_not_completion() -> None:
    assert causal_confirmation(True, True, False)["confirmed"] is False
    assert causal_confirmation(True, True, True) == {
        "classification": "RB-4",
        "confidence": "HIGH",
        "confirmed": True,
    }
    stalled = {"JogPos": "1", "DesPos": "0", "Pos": "0", "ActPos": "0"}
    assert motion_completed(stalled, "1", True) is False


def test_write_audit_separates_motion_from_configuration() -> None:
    assert classify_pmac_write("#1j=1") == "motion"
    assert classify_pmac_write("#1J/") == "close_loop_or_jog_stop"
    assert classify_pmac_write("Motor[1].ServoCtrl=1") == "activation"
    assert classify_pmac_write("P100=18018") == "test_variable"
    assert classify_pmac_write("Motor[1].JogTa=-10") == "configuration"
    assert classify_pmac_write("Sys.SimConfigOk=1") == "configuration"


def test_negative_jog_time_without_historical_value_is_not_enough() -> None:
    assert jog_time_parameter_class(NOT_MEASURED, True) == "NOT_ENOUGH_EVIDENCE"


def test_mixed_completion_inside_one_timeline_is_intermittent() -> None:
    before = {"JogPos": "0", "DesPos": "0", "Pos": "0", "ActPos": "0"}
    samples = [
        {"JogPos": "1", "DesPos": "0", "Pos": "0", "ActPos": "0"},
        {"JogPos": "1", "DesPos": "1", "Pos": "1", "ActPos": "1"},
    ]
    assert classify_pipeline(before, samples, "1") == "RB-5"
