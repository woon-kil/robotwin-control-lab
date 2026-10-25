"""P-017 Mac → Windows portproxy → Power PMAC Simulator round-trip probe.

비밀번호는 PMAC_SSH_PASSWORD 환경변수로만 받는다. 파일에 쓰지 않는다.
latency는 Mac에서 SSH 명령을 시작한 시각부터 READ 응답을 파싱한 시각까지다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import re
import stat
import subprocess
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "evidence" / "P-017"
RAW = EVIDENCE / "raw"
HOST = "192.168.0.146"
PORT = 22022
TARGET = "172.20.0.200"
VARIABLE = "P100"
BASE_VALUE = 17020
LATENCY_DEFINITION = "Mac client command start → parsed PMAC response received"
READ_RE = re.compile(rf"{VARIABLE}\s*=\s*([+-]?\d+(?:\.\d+)?)", re.IGNORECASE)
MARKER = "__P017_READ__"


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (percent / 100.0)
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def redact(text: str) -> str:
    secret = os.environ.get("PMAC_SSH_PASSWORD", "")
    if secret:
        text = text.replace(secret, "[REDACTED]")
    return text


def askpass_script() -> Path:
    handle = tempfile.NamedTemporaryFile(prefix="p017-askpass-", suffix=".sh", delete=False)
    path = Path(handle.name)
    handle.write(b"#!/bin/sh\nprintf '%s\\n' \"$PMAC_SSH_PASSWORD\"\n")
    handle.close()
    path.chmod(stat.S_IRUSR | stat.S_IXUSR)
    return path


def remote_command(value: int) -> str:
    assignment = f"{VARIABLE}={value}"
    return f"echo {assignment} | gpascii; echo {MARKER}; echo {VARIABLE} | gpascii"


def run_ssh(value: int, timeout_s: float) -> tuple[str, str, float, str | None]:
    user = os.environ.get("PMAC_SSH_USER", "root")
    askpass = askpass_script()
    env = os.environ.copy()
    env["SSH_ASKPASS"] = str(askpass)
    env["SSH_ASKPASS_REQUIRE"] = "force"
    env.setdefault("DISPLAY", ":0")
    command = [
        "ssh",
        "-p",
        str(PORT),
        "-o",
        "StrictHostKeyChecking=yes",
        "-o",
        "IdentitiesOnly=yes",
        "-o",
        "ConnectTimeout=8",
        "-o",
        "LogLevel=ERROR",
        f"{user}@{HOST}",
        remote_command(value),
    ]
    if env.get("PMAC_SSH_PASSWORD"):
        command[1:1] = ["-o", "PreferredAuthentications=password", "-o", "NumberOfPasswordPrompts=1"]
    else:
        command[1:1] = ["-o", "BatchMode=yes", "-o", "PreferredAuthentications=publickey"]
    started = time.perf_counter()
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            env=env,
            stdin=subprocess.DEVNULL,
        )
    except subprocess.TimeoutExpired as exc:
        elapsed = time.perf_counter() - started
        stdout = redact(exc.stdout or "" if isinstance(exc.stdout, str) else "")
        return stdout, "timeout", elapsed, "timeout"
    finally:
        askpass.unlink(missing_ok=True)
    elapsed = time.perf_counter() - started
    stdout = redact(completed.stdout or "")
    stderr = redact(completed.stderr or "")
    combined = stdout + stderr
    if "Permission denied" in combined:
        return combined, "auth", elapsed, "auth"
    if MARKER not in stdout and completed.returncode != 0:
        return combined, "ssh_error", elapsed, "ssh_error"
    return stdout, "", elapsed, None


def parse_read(stdout: str) -> str | None:
    section = stdout.split(MARKER, 1)[1] if MARKER in stdout else stdout
    matches = READ_RE.findall(section)
    if not matches:
        return None
    return matches[-1]


def values_match(expected: int, actual: str | None) -> bool:
    if actual is None:
        return False
    try:
        return int(float(actual)) == expected
    except ValueError:
        return False


def classify(stdout: str, transport_error: str | None, expected: int) -> tuple[bool, bool, bool, str | None, str | None]:
    if transport_error == "timeout":
        return False, False, False, None, "timeout"
    if transport_error:
        return False, False, False, None, transport_error
    actual = parse_read(stdout)
    if actual is None:
        return True, False, False, None, "parse_error"
    matched = values_match(expected, actual)
    if matched:
        return True, True, True, actual, None
    return True, True, False, actual, "mismatch"


def summarize(rows: list[dict[str, object]]) -> dict[str, object]:
    latencies = [float(row["elapsed_ms"]) for row in rows if row["error_type"] is None]
    return {
        "attempts": len(rows),
        "write_success": sum(1 for row in rows if row["write_success"]),
        "read_success": sum(1 for row in rows if row["read_success"]),
        "roundtrip_success": sum(1 for row in rows if row["roundtrip_success"]),
        "mismatch_count": sum(1 for row in rows if row["error_type"] == "mismatch"),
        "timeout_count": sum(1 for row in rows if row["error_type"] == "timeout"),
        "error_count": sum(1 for row in rows if row["error_type"] not in (None, "mismatch", "timeout")),
        "latency_definition": LATENCY_DEFINITION,
        "latency_sample": "iterations whose error_type is null",
        "latency_method": "linear interpolation on sorted successful elapsed times",
        "mean_ms": (sum(latencies) / len(latencies)) if latencies else None,
        "p50_ms": percentile(latencies, 50),
        "p95_ms": percentile(latencies, 95),
        "p99_ms": percentile(latencies, 99),
        "max_ms": max(latencies) if latencies else None,
    }


def write_evidence(rows: list[dict[str, object]], started_at: str) -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    stats = summarize(rows)
    user = os.environ.get("PMAC_SSH_USER", "root")
    credential = "runtime environment variable PMAC_SSH_PASSWORD; value not recorded"
    if not os.environ.get("PMAC_SSH_PASSWORD"):
        credential = "publickey BatchMode; no password recorded"
    environment = {
        "started_at": started_at,
        "client": "MacBook",
        "client_os": platform.platform(),
        "server_forwarder": "Windows notebook",
        "windows_address": HOST,
        "forwarding_port": PORT,
        "target": "Official Power PMAC Simulator 1.2.1",
        "target_pmac_address": TARGET,
        "interface": "SSH + gpascii",
        "ssh_user": user,
        "variable": VARIABLE,
        "credential": credential,
    }
    (EVIDENCE / "roundtrip_results.json").write_text(
        json.dumps({"environment": environment, "summary": stats, "iterations": rows}, indent=2) + "\n",
        encoding="utf-8",
    )
    (EVIDENCE / "latency_results.json").write_text(
        json.dumps(
            {
                "label": "PROJECT COMMUNICATION SPIKE LATENCY",
                "definition": LATENCY_DEFINITION,
                "not": ["network latency", "controller real-time latency", "servo response latency"],
                "mean_ms": stats["mean_ms"],
                "p50_ms": stats["p50_ms"],
                "p95_ms": stats["p95_ms"],
                "p99_ms": stats["p99_ms"],
                "max_ms": stats["max_ms"],
                "sample_count": len([row for row in rows if row["error_type"] is None]),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (EVIDENCE / "write_read_results.json").write_text(
        json.dumps({"variable": VARIABLE, "summary": stats}, indent=2) + "\n",
        encoding="utf-8",
    )
    first = rows[0]
    connected = first["error_type"] not in ("auth", "ssh_error", "timeout")
    (EVIDENCE / "local_connection.json").write_text(
        json.dumps(
            {
                "status": "PASS" if connected else "FAIL",
                "path": f"MacBook -> {HOST}:{PORT} -> {TARGET}:22 -> gpascii",
                "first_iteration_error": first["error_type"],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (EVIDENCE / "cross_host_results.json").write_text(
        json.dumps(
            {
                "status": "MEASURED" if connected else "FAIL",
                "client": "MacBook",
                "windows_address": HOST,
                "forwarding_port": PORT,
                "simulator_listen_address": TARGET,
                "port": 22,
                "protocol": "SSH + gpascii",
                "firewall_rule": "not recorded",
                "connection_result": "PASS" if connected else first["error_type"],
                "note": "This run is MacBook to the Windows portproxy. It is not the Ubuntu p620 path.",
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (RAW / "roundtrip.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    (RAW / "connection.txt").write_text(
        redact(
            f"started_at={started_at}\nuser={user}\nhost={HOST}\nport={PORT}\n"
            f"first_error={first['error_type']}\ncredential_recorded=no\n"
        ),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    if args.iterations < 1:
        raise SystemExit("iterations must be positive")
    started_at = datetime.now(timezone.utc).isoformat()
    rows: list[dict[str, object]] = []
    for index in range(args.iterations):
        expected = BASE_VALUE + index
        stdout, _transport, elapsed, transport_error = run_ssh(expected, args.timeout)
        write_ok, read_ok, round_ok, actual, error_type = classify(stdout, transport_error, expected)
        rows.append(
            {
                "iteration": index + 1,
                "test_value": expected,
                "returned_value": None if actual is None else actual,
                "write_success": write_ok,
                "read_success": read_ok,
                "roundtrip_success": round_ok,
                "success": round_ok,
                "elapsed_ms": elapsed * 1000.0,
                "error_type": error_type,
            }
        )
        if error_type == "auth":
            break
    write_evidence(rows, started_at)
    print(json.dumps(summarize(rows), indent=2))


if __name__ == "__main__":
    main()
