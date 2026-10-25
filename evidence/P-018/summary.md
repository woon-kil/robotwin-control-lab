# P-018 summary

측정: 2026-10-25T11:02:02.085535+00:00

이 문서는 P-018 FINAL PASS 또는 GO를 확정하지 않는다. 최종 판정은 별도 review다. P-019는 시작하지 않는다.

기준선: `3e68ee8fa879618ab3c16a3c312eed4c9c6d6925`

P-018 controller state represents communication/integration state only. It does not claim verified PMAC servo or motor state.

허용 범위의 표현:

> RoboTwin PowerPmacControllerAdapter was integrated with Official Power PMAC Simulator through SSH + gpascii.

이 측정이 말하지 않는 것: real Power PMAC hardware, PMAC servo control, real motor control, EtherCAT, industrial hard real-time, robot motion, production-ready PMAC integration.

## Live

경로: MacBook → `192.168.0.146:22022` → `172.20.0.200:22` → `gpascii`

| 항목 | 측정 |
|---|---|
| connect | communication_state HEALTHY, controller_state CONNECTED |
| health probe | Sys.ServoCount=2388872. servo state로 해석하지 않음 |
| P100 read | 17119 |
| P100 write | 18018 |
| P100 read-back | 18018 |
| expected == actual | true |
| disconnect | communication_state DISCONNECTED |
| post-disconnect | CommandResult REJECTED / DISCONNECTED |
| post-disconnect read | PowerPmacError DISCONNECTED |
| motion command | 보내지 않음 |

정본: `evidence/P-018/live_connection.json`, `evidence/P-018/write_read_results.json`, `evidence/P-018/adapter_results.json`

## Failure mapping

`evidence/P-018/failure_mapping.json`

authentication failure, connection refused, timeout, malformed response, disconnected operation, read-back mismatch. subprocess 예외는 transport 밖으로 나가지 않았다.

## Regression

`python3 -m pytest -q tests/unit`

95 passed, failed 0, 51.27s. `evidence/P-001`–`P-017`은 이 실행에서 다시 쓰지 않는다.

`IRobotController`, `VirtualPmacController`, `ExecutionGate`, Validator는 수정하지 않았다.
