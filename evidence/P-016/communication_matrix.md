# P-016 communication matrix

조회일: 2026-10-25. hardware에서 된다는 이유만으로 Simulator 칸을 채우지 않는다. 측정값 없음.

## SSH

- Interface: SSH
- Official support: Simulator 1.2.0 history에 기능으로 적힘
- Simulator support: 이름만 확인. gpascii, port, bind 주소는 release note에 없음
- Host OS: Simulator는 Windows setup. client OS는 미기재
- Read variable: 미확인
- Write variable: 미확인
- Issue command: 미확인
- Read motor position: 미확인
- Read controller state: 미확인
- Fault detection: 미확인
- Reconnect: 미확인
- Latency measurable: 채널이 열리면 round-trip을 잴 수 있음. 현재 측정 없음. hard real-time 아님
- Licensing/dependency: Simulator 라이선스 동의. client library는 공식 지정 없음
- Evidence: https://www.ia.omron.com/product/tool/pmac-software/history.htm

## SFTP

- Interface: SFTP
- Official support: Simulator 1.2.0 history
- Simulator support: 이름만 확인. project download가 SFTP인지는 미확인
- Host OS: Windows Simulator. client OS 미기재
- Read variable: 파일 전송이지 변수 read로 확인되지 않음
- Write variable: 미확인
- Issue command: 미확인
- Read motor position: 미확인
- Read controller state: 미확인
- Fault detection: 미확인
- Reconnect: 미확인
- Latency measurable: 명령 지연의 근거 아님
- Licensing/dependency: Simulator 라이선스
- Evidence: 같은 history

## HTTP

- Interface: HTTP
- Official support: Simulator 1.2.0 history
- Simulator support: 이름만 확인. URL과 명령 API는 미확인
- Host OS: 미기재
- Read variable: 미확인
- Write variable: 미확인
- Issue command: 미확인
- Read motor position: 미확인
- Read controller state: 미확인
- Fault detection: 미확인
- Reconnect: 미확인
- Latency measurable: 미측정
- Licensing/dependency: Simulator 라이선스
- Evidence: 같은 history

## OPC UA

- Interface: OPC UA
- Official support: Simulator 1.2.0 history. IDE 4.6.4.23이 Power PMAC의 OPC server 연결을 설정. 최소 firmware 2.8.1
- Simulator support: release note는 지원이라고 적음. node map과 외부 client 예제는 이 조사에서 확인하지 않음
- Host OS: 서버는 Simulator. client OS 미기재
- Read variable: 문서만으로 미확정
- Write variable: 문서만으로 미확정
- Issue command: 문서만으로 미확정
- Read motor position: 문서만으로 미확정
- Read controller state: 문서만으로 미확정
- Fault detection: 미확인
- Reconnect: 미확인
- Latency measurable: 미측정
- Licensing/dependency: firmware 짝과 라이선스
- Evidence: history의 IDE 4.6.4.23과 Simulator 1.2.0 항목

## MQTT

- Interface: MQTT
- Official support: Simulator 1.2.0 history. IDE 4.6.4.23은 Power PMAC이 외부 broker로 나가는 MQTT client
- Simulator support: 프로토콜 이름은 지원. RoboTwin이 broker 없이 command를 넣는 구조는 아님
- Host OS: broker 위치가 별도
- Read variable: 미확인
- Write variable: client publish라면 방향이 반대일 수 있음. 미확정
- Issue command: 미확인
- Read motor position: 미확인
- Read controller state: 미확인
- Fault detection: 미확인
- Reconnect: 미확인
- Latency measurable: 미측정
- Licensing/dependency: firmware 2.8.1 이상 문구는 IDE 항목
- Evidence: history

## DART

- Interface: DART, Direct Access Receive and Transit
- Official support: IDE history의 DART 지원. Simulator 1.2.0 history에 이름
- Simulator support: 이름만 확인. 명령 형식 미확인
- Host OS: 미기재
- Read variable: 미확인
- Write variable: 미확인
- Issue command: 미확인
- Read motor position: 미확인
- Read controller state: 미확인
- Fault detection: 미확인
- Reconnect: 미확인
- Latency measurable: 미측정
- Licensing/dependency: 미확인
- Evidence: history

## PDK

- Interface: Power PMAC PDK 4.6.4.4, .NET
- Official support: Power PMAC과 통신하는 HMI component
- Simulator support: Unknown. 다운로드 페이지가 Simulator를 이 패키지에 연결하지 않음
- Host OS: .NET component. Windows 가능성만 있고 Simulator 문서는 없음
- Read variable: Unknown
- Write variable: Unknown
- Issue command: Unknown
- Read motor position: Unknown
- Read controller state: Unknown
- Fault detection: Unknown
- Reconnect: Unknown
- Latency measurable: 미측정
- Licensing/dependency: 별도 패키지와 라이선스 동의
- Evidence: https://www.ia.omron.com/product/tool/pmac-software/

## gpascii

- Interface: gpascii online command
- Official support: Power PMAC의 script command 채널로 IDE Terminal이 존재
- Simulator support: 사용자 글에 Simulator에서 gpascii를 썼다는 문장이 있음. Omron release note는 gpascii를 따로 적지 않음. SSH 지원과 합쳐도 확정 아님
- Host OS: 미확인
- Read variable: 후보일 뿐
- Write variable: 후보일 뿐
- Issue command: 후보일 뿐
- Read motor position: 후보일 뿐
- Read controller state: 후보일 뿐
- Fault detection: 미확인
- Reconnect: 미확인
- Latency measurable: 미측정
- Licensing/dependency: 미확인
- Evidence: https://forums.automation.omron.com/topic/10813-cant-add-encoder-in-simulator/

## PComm / 기타

PComm은 이 다운로드 페이지의 Power PMAC Simulator 기능 목록에 없다. raw TCP command port, 공유 메모리, LAN bind 여부도 확인하지 않았다. Unknown이며 Supported가 아니다.

## spike에서 먼저 볼 채널

문서에 이름이 있고 변수 read/write 후보가 되는 것은 SSH와 OPC UA다. MQTT는 방향이 client-to-broker라 첫 후보가 아니다. 어느 쪽이든 P-016에서 세션을 열지 않았다.
