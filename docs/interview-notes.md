# Interview notes

RoboTwin Control Lab 면접용이다. Power PMAC 문장은 Official Power PMAC Simulator에만 적용한다. 요구사항과 architecture 계약은 바꾸지 않는다.

## 30초

RoboTwin에서 Controller 인터페이스를 분리하고 실제 산업용 controller 계열과의 integration 가능성을 보여주기 위해 Official Power PMAC Simulator adapter를 만들었습니다. 단순 명령 전송이 아니라 state read-back과 Target-vs-Actual을 Digital Twin에 연결했고, 100회 반복 실행과 5회 disconnect/reconnect를 검증했습니다.

이어서 한 문장으로 경계를 말한다. Motor[1]은 software virtual axis이고, UR10e joint로 mapping하지 않았다. Employment MVP의 실행 경로는 `VirtualPmacController`에 남아 있다.

## 실패 질문

중간에 command ACK는 왔지만 실제 target state가 변하지 않는 문제가 있었습니다. ACK를 성공으로 처리하지 않았기 때문에 Gate를 HOLD했고, JogPos와 DesPos/Pos/ActPos 사이를 분리해서 조사했습니다. root cause는 확정하지 않았으며, 이후 정상 lifecycle을 처음부터 다시 검증한 뒤 PASS했습니다.

말하지 않는 결론:

```text
root cause was Xenomai
Power PMAC bug를 해결했다
Simulator activation bug를 수정했다
```

Root cause remained NOT_CONFIRMED.

## Runtime incident

P-021 조사와 별도로, P-022 전에 Simulator 화면에서 Xenomai `rtitask` suspend 메시지가 반복되고 SSH가 끊긴 일이 있었다. 분류는 `SIMULATOR_RUNTIME_FAILURE`이고 root cause는 `NOT_CONFIRMED`다. Clean restart 후 baseline을 다시 읽고 P-022를 재개했다.

## 숫자

| 말할 수 있는 값 | 말하지 않는 해석 |
| --- | --- |
| P-022 live 3/3 | robot accuracy |
| P-023 motion 100/100 | industrial reliability |
| reconnect 5/5, false submission 0 | hardware reliability |
| tracking error 0 PMAC motor unit | 0 deg, 0 rad, 0 mm |
| elapsed mean 6.124 s, P95 6.685 s | servo latency, real-time |
| regression 197 passed | P-023 baseline 밖의 새 성능 주장 |

Elapsed는 SSH, Windows portproxy, gpascii, polling이 포함된 observed integration elapsed time이다.

## 자주 오는 질문

**Real Power PMAC를 검증했는가.** 아니다. Official Simulator다.

**UR10e를 PMAC로 움직였는가.** 아니다. Motor[1]과 robot joint mapping은 `NOT_DEFINED`다.

**100/100은 현장 신뢰성인가.** 아니다. Simulator integration evidence다. P-011의 500-cycle project workload와도 별도다.

**ACK가 오면 끝난 것인가.** 아니다. DesPos, Pos, ActPos read-back이 target과 같아야 COMPLETED다.

**ExecutionGate가 PMAC로 명령을 보내는가.** 아니다. Employment 실행은 `VirtualPmacController`다. Adapter seam은 ExecutionGate가 호출하지 않는다.
