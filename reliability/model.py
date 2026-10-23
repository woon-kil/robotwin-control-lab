"""P-011 기록 형식. Controller vocabulary는 바꾸지 않는다."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = Path(__file__).resolve().parent / "config" / "default.json"
OUTPUT = ROOT / "evidence" / "P-011"

SEQUENCE = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")
EXPECTED = {
    "A-zero": "SAFE_TO_ATTEMPT",
    "B-positive-1": "SAFE_TO_ATTEMPT",
    "C-mixed-1": "SAFE_TO_ATTEMPT",
    "E-fixed-1": "REJECT",
    "E-fixed-3": "SAFE_TO_ATTEMPT",
}
NONZERO_SAFE = ("B-positive-1", "C-mixed-1", "E-fixed-3")
# settling_time subtracts two project-clock timestamps. The clock is a running sum of 0.01 s,
# so a long session changes that subtraction by about 1e-13 s. One extra sample is 0.01 s.
DRIFT_SETTLING_ABS_SEC = 1e-9
FAILURE_CATEGORIES = (
    "VALIDATION_MISMATCH",
    "UNSAFE_SUBMISSION",
    "SAFE_NOT_SUBMITTED",
    "EXECUTION_FAILURE",
    "NOT_SETTLED",
    "TIMEOUT",
    "NON_FINITE",
    "STATE_LEAK",
    "RESET_FAILURE",
    "UNEXPECTED_FAULT",
    "METRIC_FAILURE",
    "INTERNAL_ERROR",
)
CRITICAL = frozenset({"UNSAFE_SUBMISSION"})
CLAIM = "500-cycle deterministic project workload validation"


def load_reliability_config(path: Path | None = None) -> dict:
    source = CONFIG_PATH if path is None else path
    payload = json.loads(source.read_text(encoding="utf-8"))
    sequence = tuple(payload["sequence"])
    iterations = int(payload["iterations"])
    total = int(payload["total_cycles"])
    if sequence != SEQUENCE or iterations * len(sequence) != total or total != 500:
        raise ValueError("reliability workload must stay the 5-case sequence repeated to 500")
    if payload["fault_injection_in_primary"] is not False:
        raise ValueError("primary workload does not inject faults")
    if int(payload["recovery_attempts"]) < 10:
        raise ValueError("recovery suite attempts at least 10")
    return payload
