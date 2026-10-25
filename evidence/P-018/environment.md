# P-018 environment

측정: 2026-10-25T11:02:02.085535+00:00

```text
client: MacBook
client_os: macOS-26.6.2-arm64-arm-64bit-Mach-O
server/forwarder: Windows notebook
Windows address: 192.168.0.146
forwarding port: 22022
target: Official Power PMAC Simulator 1.2.1
target PMAC address: 172.20.0.200
interface: SSH + gpascii
variables: Sys.ServoCount, P100
credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded
```

P-018 controller state represents communication/integration state only. It does not claim verified PMAC servo or motor state.

Sys.ServoCount 값은 통신 health를 확인한 파싱 결과다. servo enable이나 motor state가 아니다.
