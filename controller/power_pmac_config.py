"""P-018 test endpoint.

이 주소는 P-017에서 측정한 개발용 경로다. production host 가정이 아니다.
password 값은 이 모듈에 두지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PowerPmacConfig:
    host: str
    port: int
    user: str
    simulator_address: str
    password_env: str = "PMAC_SSH_PASSWORD"
    timeout_s: float = 20.0
    connect_timeout_s: int = 8


P017_TEST_CONFIG = PowerPmacConfig(
    host="192.168.0.146",
    port=22022,
    user="root",
    simulator_address="172.20.0.200",
)
