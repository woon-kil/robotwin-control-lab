# P-023 environment

측정: 2026-10-26T00:10:54.285228+00:00

client: MacBook
path: MacBook -> 192.168.0.146:22022 -> Windows portproxy -> 172.20.0.200:22
hostname: ppmac
os: Linux
eth0: 2: eth0    inet 172.20.0.200/24 brd 172.20.0.255 scope global eth0\       valid_lft forever preferred_lft forever
pmac backend: Official Power PMAC Simulator 1.2.1
axis: software virtual Motor[1]
unit: pmac_motor_unit
credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded

SSH + Windows portproxy + gpascii + polling을 포함한 observed integration elapsed time. servo latency, controller cycle time, real-time latency, PMAC execution latency가 아니다.

P-011 500-cycle evidence와 이 측정은 별개다.
industrial reliability는 주장하지 않는다.
