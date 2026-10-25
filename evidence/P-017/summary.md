# P-017 summary

측정: 2026-10-25T08:36:23Z

Gate 판정: **BLOCKED**

GO / CONDITIONAL GO / 측정된 NO-GO가 아니다. Simulator가 없어 통신이 실패했다고 쓰지 않는다. P-018로 가지 않는다.

| 항목 | 판정 |
|---|---|
| Official Simulator 실행 | BLOCKED |
| 외부 client connection | NOT RUN |
| External WRITE | NOT RUN |
| External READ | NOT RUN |
| Write→Read match | NOT RUN |
| Test variable read | NOT RUN |
| Virtual motor position read | NOT RUN |
| Physical encoder read | NOT TESTED |
| Round-trip | NOT RUN |
| Response parsing | NOT RUN |
| Timeout/error 식별 | NOT RUN |
| Disconnect/reconnect | NOT RUN |
| Cross-host Ubuntu→Windows | BLOCKED |
| Automation | NO |
| GUI dependency | 세션이 없어 해당 없음 |

Round-trip attempts는 0이다. success, failure, timeout은 측정값이 아니다.

Latency는 PROJECT COMMUNICATION SPIKE LATENCY로 둘 자리였으나 표본이 없다. mean, P50, P95, P99, max는 null이다. PMAC real-time latency가 아니다.

RoboTwin core 수정: NO  
`evidence/P-001`–`P-016` 수정: NO  
`PowerPmacControllerAdapter`: 작성하지 않음
