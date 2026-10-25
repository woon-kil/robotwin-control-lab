# P-017 — Automated Power PMAC Communication Probe

측정 시각: 2026-10-25T10:41:54.742760+00:00

이 문서는 GO / PASS / FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-018은 시작하지 않는다.

2026-10-25T08:36:23Z의 environment probe는 Windows Simulator를 찾지 못해 BLOCKED였다. 아래는 그 이후 MacBook에서 연 통신 측정이다.

## 경로

```text
client: MacBook
server/forwarder: Windows notebook
Windows address: 192.168.0.146
forwarding port: 22022
target: Official Power PMAC Simulator 1.2.1
target PMAC address: 172.20.0.200
interface: SSH + gpascii
variable: P100
```

MacBook → `192.168.0.146:22022` → Windows portproxy → `172.20.0.200:22` → `gpascii`

credential은 `PMAC_SSH_PASSWORD` 환경변수로만 넘겼다. 값은 source, config, evidence, log에 없다.

## 실행

```text
python3 spikes/p017_vppmac/roundtrip_probe.py --iterations 100
```

각 iteration은 `P100`에 `17020 + index`를 WRITE한 뒤 READ하고, marker `__P017_READ__` 뒤의 마지막 `P100=<number>`를 expected와 비교한다. iteration 1은 17020, iteration 100은 17119다.

## 측정

정본은 `evidence/P-017/roundtrip_results.json`이다.

| 항목 | 값 |
|---|---|
| attempts | 100 |
| write_success | 100 |
| read_success | 100 |
| roundtrip_success | 100 |
| mismatch_count | 0 |
| timeout_count | 0 |
| error_count | 0 |

Latency label: `PROJECT COMMUNICATION SPIKE LATENCY`

정의: `Mac client command start → parsed PMAC response received`

SSH handshake와 portproxy가 포함된다. network latency, controller real-time latency, servo response latency가 아니다.

| 통계 | ms |
|---|---|
| mean | 3943.166506662965 |
| P50 | 3674.0326459985226 |
| P95 | 5704.727576370351 |
| P99 | 6505.163115102344 |
| max | 8779.493291862309 |

iteration별 원문은 `evidence/P-017/raw/roundtrip.jsonl`이다.

## 하지 않은 것

- `IRobotController`, `VirtualPmacController`, `ExecutionGate`, Validator 수정
- `PowerPmacControllerAdapter` 구현
- PMAC motion program, motor motion command
- `Motor[x].ActPos` read
- disconnect / reconnect
- Ubuntu p620 → Windows
- real Power PMAC hardware 검증
- real-time 성능 주장
