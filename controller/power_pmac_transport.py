"""SSH + gpascii transport for the P-018 adapter.

Transport는 harmless variable만 읽고 쓴다. motor command를 만들지 않는다.
subprocess 예외는 PowerPmacError로 바뀌고 밖으로 나가지 않는다.
"""

from __future__ import annotations

import os
import re
import stat
import subprocess
import tempfile
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from controller.power_pmac_config import PowerPmacConfig

MARKER = "__P018_READ__"
READ_NAMES = frozenset({"P100", "Sys.ServoCount"})
WRITE_NAMES = frozenset({"P100"})
_CONNECTION_MARKERS = (
    "connection refused",
    "no route to host",
    "network is unreachable",
    "could not resolve",
    "connection timed out",
    "operation timed out",
    "host key verification failed",
)
Runner = Callable[[list[str], dict[str, str], float], subprocess.CompletedProcess[str]]


class PowerPmacTransport(Protocol):
    def read_variable(self, name: str, timeout_s: float | None = None) -> str: ...

    def write_and_read(self, name: str, value: int, timeout_s: float | None = None) -> str: ...

    def close(self) -> None: ...


class PowerPmacError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class VariableRead:
    name: str
    value: int
    text: str


def require_read_name(name: str) -> None:
    if name not in READ_NAMES:
        raise PowerPmacError("PROTOCOL_ERROR", "variable is outside the P-018 harmless set")


def require_write(name: str, value: object) -> None:
    require_read_name(name)
    if name not in WRITE_NAMES:
        raise PowerPmacError("PROTOCOL_ERROR", "P-018 write is limited to P100")
    if isinstance(value, bool) or not isinstance(value, int):
        raise PowerPmacError("PROTOCOL_ERROR", "P100 value must be an int")


def parse_integer(token: str) -> int:
    if re.fullmatch(r"[+-]?\d+", token):
        return int(token)
    if re.fullmatch(r"[+-]?\d+\.0+", token):
        return int(token.split(".", 1)[0])
    raise PowerPmacError("PROTOCOL_ERROR", "PMAC value was not an integer")


def interpret(stdout: str, stderr: str, returncode: int, variable: str, marker: str | None) -> str:
    combined = stdout + "\n" + stderr
    if "Permission denied" in combined:
        raise PowerPmacError("AUTHENTICATION_FAILURE", "SSH authentication failed")
    section = stdout
    if marker is not None:
        if marker not in stdout:
            _raise_unparsed(combined, returncode)
        section = stdout.split(marker, 1)[1]
    matches = re.findall(rf"{re.escape(variable)}\s*=\s*([+-]?\d+(?:\.\d+)?)", section)
    if not matches:
        _raise_unparsed(combined, returncode)
    return matches[-1]


def redact(text: str, secret: str) -> str:
    if secret:
        return text.replace(secret, "[REDACTED]")
    return text


class ScriptedTransport:
    """테스트 대역. Simulator에 접속하지 않는다."""

    def __init__(self) -> None:
        self._reads: list[str | PowerPmacError] = []
        self._write_reads: list[str | PowerPmacError] = []
        self.calls: list[tuple[object, ...]] = []
        self.closed = False

    def queue_read(self, item: str | PowerPmacError) -> None:
        self._reads.append(item)

    def queue_write_read(self, item: str | PowerPmacError) -> None:
        self._write_reads.append(item)

    def read_variable(self, name: str, timeout_s: float | None = None) -> str:
        require_read_name(name)
        self.calls.append(("read", name, timeout_s))
        return _next(self._reads, "read")

    def write_and_read(self, name: str, value: int, timeout_s: float | None = None) -> str:
        require_write(name, value)
        self.calls.append(("write_read", name, value, timeout_s))
        return _next(self._write_reads, "write_read")

    def close(self) -> None:
        self.closed = True
        self.calls.append(("close",))


class SshGpasciiTransport:
    def __init__(self, config: PowerPmacConfig, runner: Runner | None = None) -> None:
        self._config = config
        self._runner = runner or _default_runner

    def read_variable(self, name: str, timeout_s: float | None = None) -> str:
        require_read_name(name)
        stdout, stderr, returncode = self._run(f"echo {name} | gpascii", timeout_s)
        return interpret(stdout, stderr, returncode, name, None)

    def write_and_read(self, name: str, value: int, timeout_s: float | None = None) -> str:
        require_write(name, value)
        remote = f"echo {name}={value} | gpascii; echo {MARKER}; echo {name} | gpascii"
        stdout, stderr, returncode = self._run(remote, timeout_s)
        return interpret(stdout, stderr, returncode, name, MARKER)

    def close(self) -> None:
        return None

    def _run(self, remote: str, timeout_s: float | None) -> tuple[str, str, int]:
        timeout = self._config.timeout_s if timeout_s is None else timeout_s
        command = _ssh_command(self._config, remote)
        env = os.environ.copy()
        secret = env.get(self._config.password_env, "")
        askpass: Path | None = None
        if secret:
            askpass = _askpass_script(self._config.password_env)
            env["SSH_ASKPASS"] = str(askpass)
            env["SSH_ASKPASS_REQUIRE"] = "force"
            env.setdefault("DISPLAY", ":0")
        try:
            try:
                completed = self._runner(command, env, timeout)
            except subprocess.TimeoutExpired:
                raise PowerPmacError("TIMEOUT", "SSH command timed out") from None
            except OSError:
                raise PowerPmacError("CONNECTION_UNREACHABLE", "SSH client could not start") from None
        finally:
            if askpass is not None:
                askpass.unlink(missing_ok=True)
        stdout = completed.stdout if isinstance(completed.stdout, str) else ""
        stderr = completed.stderr if isinstance(completed.stderr, str) else ""
        return redact(stdout, secret), redact(stderr, secret), int(completed.returncode)


def _ssh_command(config: PowerPmacConfig, remote: str) -> list[str]:
    command = [
        "ssh",
        "-p",
        str(config.port),
        "-o",
        "StrictHostKeyChecking=yes",
        "-o",
        "IdentitiesOnly=yes",
        "-o",
        f"ConnectTimeout={config.connect_timeout_s}",
        "-o",
        "LogLevel=ERROR",
    ]
    if os.environ.get(config.password_env):
        command.extend(["-o", "PreferredAuthentications=password", "-o", "NumberOfPasswordPrompts=1"])
    else:
        command.extend(["-o", "BatchMode=yes", "-o", "PreferredAuthentications=publickey"])
    command.append(f"{config.user}@{config.host}")
    command.append(remote)
    return command


def _default_runner(command: list[str], env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_s,
        env=env,
        stdin=subprocess.DEVNULL,
    )


def _askpass_script(env_name: str) -> Path:
    if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", env_name) is None:
        raise PowerPmacError("PROTOCOL_ERROR", "password env name is invalid")
    handle = tempfile.NamedTemporaryFile(prefix="p018-askpass-", suffix=".sh", delete=False)
    path = Path(handle.name)
    handle.write(f"#!/bin/sh\nprintf '%s\\n' \"${env_name}\"\n".encode())
    handle.close()
    path.chmod(stat.S_IRUSR | stat.S_IXUSR)
    return path


def _next(items: list[str | PowerPmacError], kind: str) -> str:
    if not items:
        raise PowerPmacError("PROTOCOL_ERROR", "scripted transport has no " + kind)
    item = items.pop(0)
    if isinstance(item, PowerPmacError):
        raise item
    return item


def _raise_unparsed(combined: str, returncode: int) -> None:
    lowered = combined.lower()
    if any(marker in lowered for marker in _CONNECTION_MARKERS):
        raise PowerPmacError("CONNECTION_UNREACHABLE", "SSH connection failed")
    reached_pmac = "stdin open" in lowered or "gpascii" in lowered
    if returncode != 0 and not reached_pmac:
        raise PowerPmacError("CONNECTION_UNREACHABLE", "SSH command failed before a PMAC value")
    raise PowerPmacError("PROTOCOL_ERROR", "PMAC response was not parseable")
