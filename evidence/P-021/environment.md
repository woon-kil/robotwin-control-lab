# P-021 environment

측정: 2026-10-25T13:11:17.206406+00:00

```text
client: MacBook
client_os: macOS-26.6.2-arm64-arm-64bit-Mach-O
virtual backend: VirtualPmacController existing reset_fault and servo_on
pmac backend: Official Power PMAC Simulator 1.2.1
firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable
path: MacBook -> 192.168.0.146:22022 -> 172.20.0.200:22 -> SSH + gpascii
live path: adapter disconnect and connect only; the simulator process is not stopped
live fault injection: NOT_TESTED
scripted fault: AmpFault, FeFatal, LimitStop on an exchange double
credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded
```

P-017, P-018, P-019, P-019C, P-020 evidence는 수정하지 않는다.
