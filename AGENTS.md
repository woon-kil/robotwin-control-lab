## Learned User Preferences

- 한 Gate가 끝나면 다음 Gate를 자동으로 열지 않는다. 「진행」, 「커밋」, 「푸시」는 현재 Gate 산출물의 커밋·푸시 트리거이며, 그 말만으로 다음 단계를 시작하지 않는다.
- 지정된 Gate 밖의 구현을 먼저 하지 않는다. 시뮬레이터, FK/IK, Unity, Isaac, ROS2, PMAC, HMI, Long-Reach, Bolt, Tool, Workcell은 해당 Gate가 열리기 전에 시작하지 않는다.
- 화면에서 동작하거나 AI가 제안했다는 이유만으로 PASS하지 않는다. 측정하지 않은 숫자와 출처 없는 물리 파라미터는 PASS 근거로 쓰지 않는다.
- 요구사항·아키텍처 문장과 번호는 해당 Controlled Amendment Gate 없이 바꾸지 않는다. 아직 없는 Test, Evidence, Gate를 있는 것처럼 쓰지 않는다.
- 대화와 문서는 한글로 쓴다. 기술 용어와 상태값은 영어를 유지한다.
- 완료 보고는 짧게 한다. 장문 RESULT 전문을 사용자에게 복사하도록 요구하지 않는다.

## Learned Workspace Facts

- Unity와 NVIDIA Isaac Sim에서 같은 로봇 모델과 표준 Robot Command를 실행하고, 독립 Kinematics/Validation으로 검증하며, ROS2와 PMAC-compatible virtual controller와 연동하는 산업용 로봇 Digital Twin 및 Pre-Execution Validation 플랫폼이다. 한쪽 시뮬레이터를 다른 쪽의 Viewer로 두지 않는다.
- Golden Reference는 `UniversalRobots/Universal_Robots_ROS2_Description`의 official nominal UR10e 설명이다. 로봇별 calibration 파일이 아니다. 정확한 upstream commit은 `robot_description/SOURCE_MANIFEST.md`에 핀되어 있고, 브랜치 이름만으로 핀하지 않는다. Generic Long-Reach Manipulator는 Golden Reference 검증 뒤에 확장한다.
- 대표 Use Case는 Generic Industrial Bolt Fastening / Loosening with Torque-Aware Validation이다. 정본은 `docs/REQUIREMENTS.md`의 REQ-023–REQ-028이다. Hazardous Filter Handling은 도메인 동기이며 최종 대표 데모가 아니다. Amendment 문서화 이후에도 Bolt, Torque, Tool, Workcell 구현은 별도 구현 Gate 전에 시작하지 않는다.
- Pre-Execution Validation의 PASS는 `SAFE TO ATTEMPT`이며 체결 성공이 아니다. 초기 Torque는 Virtual Fastening Process와 Virtual Torque Sensor다. HMI와 Evidence에는 simulated/virtual임을 표시한다. 실제 제품 체결 Torque는 추정하지 않고, 숫자가 필요하면 `PROJECT TEST PARAMETER` 또는 `SIMULATION TEST PARAMETER`로 적는다.
- MVP 대표 Tool은 Bolt Fastening / Loosening Tool 하나다. Cover Handling, Filter Gripper, Filter Transport, Inspection, Automatic Tool Changer는 현재 MVP에서 제외한다. Bolt Tool TCP는 Robot Flange와 별도다.
- Core Robotics FilterBot 복제, PMAC proprietary protocol 복제, 산업 안전 인증을 주장하지 않는다. 실제 PMAC 하드웨어는 전제하지 않는다. 진단과 정비 판단은 Maintenance-Agentic-Twin의 책임이며, 명시적 Integration Gate 없이 통합하지 않는다.
- 원격은 GitHub public `woon-kil/robotwin-control-lab`의 `main`이다. `.cursor/hooks/` 상태 파일은 커밋하지 않는다.
