# P-017 environment

출처: `spikes/p017_vppmac/roundtrip_probe.py`  
측정: 2026-10-25T10:41:54.742760+00:00  
정본: `evidence/P-017/roundtrip_results.json`

2026-10-25T08:36:23Z environment probe는 이 Mac과 p620만 보고 BLOCKED였다. 아래는 그 이후의 MacBook 경로다.

```text
client: MacBook
client_os: macOS-26.6.2-arm64-arm-64bit-Mach-O
server/forwarder: Windows notebook
Windows address: 192.168.0.146
forwarding port: 22022
target: Official Power PMAC Simulator 1.2.1
target PMAC address: 172.20.0.200
interface: SSH + gpascii
variable: P100
ssh user: root
credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded
```

경로는 MacBook → `192.168.0.146:22022` → Windows portproxy → `172.20.0.200:22` → `gpascii`다.

이 실행은 Simulator process를 기동하지 않았다. Ubuntu p620 경로는 측정하지 않았다. Firewall rule은 기록하지 않았다.
