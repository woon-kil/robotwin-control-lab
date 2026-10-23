# P-007 environment

이 기록은 adapter를 실행한 기계의 측정이다.

```text
PRETTY_NAME="Ubuntu 24.04.4 LTS"
NAME="Ubuntu"
VERSION_ID="24.04"
VERSION="24.04.4 LTS (Noble Numbat)"
VERSION_CODENAME=noble
ID=ubuntu
ID_LIKE=debian
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
UBUNTU_CODENAME=noble
LOGO=ubuntu-logo

python 3.12.3 (main, Aug 31 2026, 10:18:26) [GCC 13.3.0]
ROS_DISTRO jazzy
ROS_DOMAIN_ID 27
platform Linux-6.8.0-147-generic-x86_64-with-glibc2.39
rclpy /opt/ros/jazzy/lib/python3.12/site-packages/rclpy/__init__.py
```

Isaac Sim과 Unity는 이 게이트에서 다시 실행하지 않았다.
P-001B부터 P-006 evidence 파일은 이 실행이 쓰지 않는다.

Adapter stdout tail:

```text
[INFO] [1792728615.782495624] [ros2_controller_adapter]: VirtualPmacController READY
```
