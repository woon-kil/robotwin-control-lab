# RoboTwin Control Lab

Status:
P-000A — Architecture Baseline

Current Gate:
P-000A

Implementation:
NOT STARTED

Unity와 NVIDIA Isaac Sim에서 동일한 로봇 모델과 표준 Robot Command를 실행하고, 독립된 Kinematics/Validation 계층으로 결과를 검증하는 산업용 로봇 Digital Twin 및 제어 검증 플랫폼의 기준선이다.
초기 Golden Reference 후보는 UR10e 계열이다. 공식 소스와 버전은 P-000A에서 확정하지 않으며, P-001 시작 전 Source Baseline Gate에서 검증한다.
Golden Reference 검증 후 Generic Long-Reach Manipulator로 확장한다. 이 모델은 산업용 장거리 매니퓰레이션 요구에서 영감을 받은 포트폴리오 모델이며, 특정 독점 로봇의 복제가 아니다.
ROS2 및 PMAC-compatible Controller와 연동하는 구조를 목표로 한다. 실제 PMAC 하드웨어와 비공개 프로토콜은 확보되지 않은 것으로 가정한다.
현재 단계는 문서 기준선이다. 구현은 시작하지 않았다.
