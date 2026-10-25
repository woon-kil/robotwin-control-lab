"""P-017 environment probe.

Official Power PMAC Simulator가 이 실행 환경에 없으면 통신을 시도하지 않는다.
SSH/gpascii 성공을 가정하지 않고, 측정하지 않은 항목을 PASS로 쓰지 않는다.
"""

from __future__ import annotations

import json
import platform
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "evidence" / "P-017"
RAW = EVIDENCE / "raw"


def run(command: list[str], timeout: float) -> tuple[int, str]:
    try:
        completed = subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)
    text = (completed.stdout or "") + (completed.stderr or "")
    return completed.returncode, text.strip()


def main() -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    probed_at = datetime.now(timezone.utc).isoformat()
    local_code, local_text = run(["sw_vers"], 5)
    p620_code, p620_text = run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "p620", "cat /etc/os-release"],
        15,
    )
    p620_pretty = ""
    for line in p620_text.splitlines():
        if line.startswith("PRETTY_NAME="):
            p620_pretty = line.split("=", 1)[1].strip().strip('"')
    simulator_home = Path.home() / "Applications"
    names = ["PowerPMAC", "Power PMAC", "PMACSimulator"]
    local_hits = []
    search_roots = [Path("/Applications"), simulator_home, Path.home() / "Parallels"]
    for root in search_roots:
        if not root.exists():
            continue
        for path in root.iterdir():
            if any(name.lower() in path.name.lower() for name in names):
                local_hits.append(str(path))
    rdp_installed = Path("/Applications/Windows App.app").exists() or Path(
        "/Applications/Microsoft Remote Desktop.app"
    ).exists()
    vm_tools = {
        "parallels": Path("/Applications/Parallels Desktop.app").exists(),
        "utm": Path("/Applications/UTM.app").exists(),
        "virtualbox": shutil.which("VBoxManage") is not None,
    }
    environment = {
        "probed_at": probed_at,
        "local_os": platform.platform(),
        "sw_vers_returncode": local_code,
        "sw_vers": local_text,
        "p620_ssh_returncode": p620_code,
        "p620_pretty_name": p620_pretty or None,
        "windows_app_installed": rdp_installed,
        "windows_vm_tools": vm_tools,
        "local_pmac_paths": local_hits,
        "simulator_process": None,
        "status": "BLOCKED",
        "reason": "No Windows host with an installed Power PMAC Simulator was found. No protocol session was opened.",
    }
    not_run = {
        "status": "NOT RUN",
        "attempts": 0,
        "reason": environment["reason"],
        "measured": False,
    }
    (EVIDENCE / "local_connection.json").write_text(json.dumps({**not_run, "protocol": None}, indent=2) + "\n", encoding="utf-8")
    (EVIDENCE / "write_read_results.json").write_text(
        json.dumps({**not_run, "write": "NOT RUN", "read": "NOT RUN", "match": "NOT RUN"}, indent=2) + "\n",
        encoding="utf-8",
    )
    (EVIDENCE / "roundtrip_results.json").write_text(
        json.dumps(
            {
                **not_run,
                "success": None,
                "failure": None,
                "timeout": None,
                "invalid_response": None,
                "iterations": [],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (EVIDENCE / "latency_results.json").write_text(
        json.dumps(
            {
                "status": "NOT RUN",
                "label": "PROJECT COMMUNICATION SPIKE LATENCY",
                "mean_ms": None,
                "p50_ms": None,
                "p95_ms": None,
                "p99_ms": None,
                "max_ms": None,
                "reason": environment["reason"],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (EVIDENCE / "reconnect_results.json").write_text(json.dumps(not_run, indent=2) + "\n", encoding="utf-8")
    (EVIDENCE / "cross_host_results.json").write_text(
        json.dumps(
            {
                "status": "BLOCKED",
                "ubuntu_host": "p620" if p620_code == 0 else None,
                "ubuntu_os": p620_pretty or None,
                "windows_host": None,
                "simulator_listen_address": None,
                "port": None,
                "protocol": None,
                "firewall_rule": "NOT RUN",
                "connection_result": "NOT RUN",
                "reason": "Windows Power PMAC Simulator 1.2.1 was not running, so the Ubuntu path was not attempted.",
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (RAW / "simulator_start.txt").write_text(
        "BLOCKED\nPower PMAC Simulator process was not started.\n"
        f"local_pmac_paths={local_hits}\nwindows_vm_tools={vm_tools}\n",
        encoding="utf-8",
    )
    (RAW / "connection.txt").write_text("NOT RUN\nNo simulator endpoint.\n", encoding="utf-8")
    (RAW / "environment.json").write_text(json.dumps(environment, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "BLOCKED", "p620": p620_pretty, "local_hits": local_hits}, indent=2))


if __name__ == "__main__":
    main()
