# P-017 summary

측정: 2026-10-25T10:41:54.742760+00:00

이전 environment probe(2026-10-25T08:36:23Z)는 BLOCKED였다. 이번 기록은 그 실행을 대체하는 MacBook 경로 측정이다.

이 문서는 GO / PASS / FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-018은 시작하지 않는다.

| 항목 | 측정 |
|---|---|
| attempts | 100 |
| write_success | 100 |
| read_success | 100 |
| roundtrip_success | 100 |
| mismatch_count | 0 |
| timeout_count | 0 |
| error_count | 0 |
| test values | P100 = 17020 .. 17119 |
| Disconnect/reconnect | NOT RUN |
| Virtual motor `Motor[x].ActPos` | NOT RUN |
| Physical encoder | NOT TESTED |
| Ubuntu p620 → Windows | NOT RUN |

Latency label: `PROJECT COMMUNICATION SPIKE LATENCY`

정의: `Mac client command start → parsed PMAC response received`

이것은 network latency도 controller real-time latency도 아니다. 표본은 `error_type`이 null인 iteration이다. 방법은 sorted elapsed time의 linear interpolation이다.

| 통계 | ms |
|---|---|
| mean | 3943.166506662965 |
| P50 | 3674.0326459985226 |
| P95 | 5704.727576370351 |
| P99 | 6505.163115102344 |
| max | 8779.493291862309 |

정본: `evidence/P-017/roundtrip_results.json`, `evidence/P-017/latency_results.json`, `evidence/P-017/raw/roundtrip.jsonl`

RoboTwin core 수정: NO  
`evidence/P-001`–`P-016` 수정: NO  
`PowerPmacControllerAdapter`: 작성하지 않음  
credential 값: 기록하지 않음
