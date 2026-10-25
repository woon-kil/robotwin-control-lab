# P-017 protocol discovery

측정: 2026-10-25T10:41:54.742760+00:00

이번 실행에서 사용한 interface는 SSH + `gpascii`다. 변수는 P100이다. motion command는 보내지 않았다.

| 질문 | 이번 실행 |
|---|---|
| Connection established? | 100/100 iteration이 READ 값을 파싱했다 |
| Authentication works? | runtime credential로 세션이 열렸다. 값은 기록하지 않음 |
| Command accepted? | `P100=<int>` WRITE 후 READ-back이 expected와 같았다 |
| Variable write works? | write_success 100 |
| Variable read works? | read_success 100 |
| Response parseable? | `P100=<number>` 를 marker `__P017_READ__` 뒤에서 파싱했다 |
| WRITE → READ-back | roundtrip_success 100, mismatch_count 0 |
| Reconnect works? | NOT RUN |

OPC UA, SFTP, HTTP, DART, MQTT는 이번 실행에서 열지 않았다. UNVERIFIED다.

`gpascii`의 process exit code는 0이 아니어도 stdout에 `P100=<number>`가 있다. probe는 그 stdout을 파싱하고, `Permission denied`만 auth로 분류한다.

이 결과는 Official Power PMAC Simulator 1.2.1에 대한 이번 SSH + gpascii 측정이다. real Power PMAC hardware 검증이 아니다.
