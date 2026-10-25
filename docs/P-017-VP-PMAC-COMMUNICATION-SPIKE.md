# P-017 — VP-PMAC Environment & Communication Spike

측정 시각: 2026-10-25T08:36:23Z

판정: **BLOCKED**

GO도 CONDITIONAL GO도 아니다. 측정된 protocol 실패가 아니므로 NO-GO도 아니다. Official Power PMAC Simulator를 실행할 Windows 호스트가 없어 통신 세션을 열지 않았다. SSH/gpascii와 OPC UA는 UNVERIFIED다. P-018은 시작하지 않는다.

## 실행한 것

`spikes/p017_vppmac/probe_environment.py`가 이 Mac과 `ssh p620`만 확인했다.

- 이 기계: macOS 26.6.2, arm64
- p620: Ubuntu 24.04.4 LTS
- Parallels, UTM, VirtualBox: 없음
- Windows App은 설치되어 있다. 저장된 Remote Desktop 대상은 없었다.
- Power PMAC IDE / Simulator 경로는 찾지 못했다.
- Simulator process는 없다.

ARP에 다른 LAN 주소가 보였다. PMAC으로 확인되지 않았으므로 스캔하지 않았고 주소는 기록하지 않았다.

## 하지 않은 것

- Simulator 기동
- 외부 WRITE / READ
- `Motor[x].ActPos` read
- 20회 round-trip
- disconnect / reconnect
- Ubuntu에서 Windows Simulator로 접속

이 항목은 PASS가 아니다. `evidence/P-017/`의 해당 파일은 `NOT RUN` 또는 `BLOCKED`다.

## Protocol

선택한 protocol 없다. P-016의 후보 SSH, SFTP, HTTP, OPC UA, DART, MQTT는 이번 실행에서 모두 UNVERIFIED다. hardware에서 gpascii가 쓰인다는 이유로 Simulator 접속을 시도하지 않았다.

## 다음

Windows 10 또는 11에 Power PMAC IDE 4.6.5.3과 Simulator 1.2.1이 설치된 뒤에만 이 spike를 다시 연다. 그때도 local Windows client가 성공한 다음 cross-host를 본다. adapter는 그때도 이 결과만으로 작성하지 않는다.
