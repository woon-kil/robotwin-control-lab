"""P-001B evidence를 FK 함수로 다시 계산해 쓴다."""

import json
import math
from pathlib import Path

from core.kinematics.configurations import baseline_configurations
from core.kinematics.fk import forward_kinematics
from core.kinematics.limits import limit_violations
from core.model.ur10e import SOURCE_COMMIT, SOURCE_MANIFEST, SOURCE_REPOSITORY, load_ur10e_model

TEST_COMMAND = "PYTHONPATH=. python3 -m pytest tests/unit -q"


def write_evidence(repo: Path | None = None) -> Path:
    root = Path(__file__).resolve().parents[2] if repo is None else repo
    model = load_ur10e_model()
    rows = []
    for item in baseline_configurations():
        joints = tuple(item["joint_vector_rad"])
        if limit_violations(model, joints):
            raise RuntimeError(f"{item['test_id']} is outside stored joint limits")
        first = forward_kinematics(model, joints)
        second = forward_kinematics(model, joints)
        finite = all(
            math.isfinite(value)
            for value in (first.x, first.y, first.z, first.qx, first.qy, first.qz, first.qw)
        )
        same = first == second
        if not finite or not same:
            raise RuntimeError(f"{item['test_id']} is not a finite deterministic pose")
        rows.append(
            {
                "test_id": item["test_id"],
                "joint_vector_rad": list(joints),
                "position_m": {"x": first.x, "y": first.y, "z": first.z},
                "quaternion_xyzw": {
                    "x": first.qx,
                    "y": first.qy,
                    "z": first.qz,
                    "w": first.qw,
                },
                "finite": finite,
                "deterministic_rerun": same,
            }
        )
    payload = {
        "gate": "P-001B",
        "source_repository": SOURCE_REPOSITORY,
        "source_commit": SOURCE_COMMIT,
        "robot_type": model.robot_id,
        "package": "ur_description",
        "source_manifest": SOURCE_MANIFEST,
        "model_identifier": model.model_identifier,
        "base_frame": model.base_frame,
        "tcp_frame": model.tcp_frame,
        "actuated_joint_order": list(model.actuated_joint_order),
        "units": {
            "position": "meter",
            "joint_angle": "radian",
            "orientation": "quaternion_xyzw",
        },
        "joint_limit_policy": "FK calculates the transform and does not clamp. Joint limits stay on the model and are checked separately.",
        "external_fk_accuracy_claim": False,
        "configurations": rows,
    }
    evidence = root / "evidence" / "P-001B"
    evidence.mkdir(parents=True, exist_ok=True)
    results_path = evidence / "fk_results.json"
    results_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    summary = _summary(payload)
    (evidence / "summary.md").write_text(summary, encoding="utf-8")
    return evidence


def _summary(payload: dict) -> str:
    order = ", ".join(payload["actuated_joint_order"])
    count = len(payload["configurations"])
    return f"""# P-001B Independent FK

```text
Gate
P-001B

upstream SHA
{payload["source_commit"]}

Result
PASS
```

이 PASS는 Canonical model이 재현되고, 독립 FK가 결정적으로 실행되며, 구조 검사가 통과했다는 뜻이다. 외부 reference와 맞다는 정확도 합격은 아니다. Unity, Isaac, 실제 로봇과 비교하지 않았다.

## Canonical model

```text
source repository
{payload["source_repository"]}

source commit
{payload["source_commit"]}

robot type
{payload["robot_type"]}

source manifest
{payload["source_manifest"]}

model identifier
{payload["model_identifier"]}

base_frame
{payload["base_frame"]}

tcp_frame
{payload["tcp_frame"]}
```

`base_frame`은 source의 `base_link`다. `base`는 같은 프레임이 아니다. `tcp_frame`은 source의 `tool0`이다. `T_tool0_tcp`와 `bolt_tool_tcp`는 숫자가 없어 곱하지 않았다.

관절 원점의 rpy는 URDF 고정축 회전 Rz(yaw) Ry(pitch) Rx(roll)이다. `T_parent_child(q) = T_origin * T_joint(q)`이고, `T_base_link_tool0`은 그 행렬을 base에서 tool0까지 곱한 값이다.

## Joint order

```text
{order}
```

이 순서는 xacro의 actuated 직렬 체인이다. dictionary 순서나 시뮬레이터 import 순서를 쓰지 않는다.

## Tests

```text
test count
{count}

test method
고정 joint vector. 난수 없음.
각 vector를 forward_kinematics에 두 번 넣어 같은 Pose인지 확인한다.
구조 검사는 tests/unit이다.

test command
{TEST_COMMAND}
```

위치 단위는 meter, 관절각은 radian, 자세 저장은 quaternion (x, y, z, w)이다. Euler는 저장하지 않는다.

Joint limit 밖의 입력도 FK가 clamp하지 않는다. 한계는 모델에 radian으로 보존되고 `limit_violations`가 따로 검사한다. source의 `!degrees`는 로드할 때 `degrees * pi / 180`으로 바뀐다.

## Known limitations

- official nominal UR10e model 기반이다.
- robot-specific calibration이 아니다.
- 실제 UR10e 측정값과 비교하지 않았다.
- Unity와 비교하지 않았다.
- Isaac Sim과 비교하지 않았다.
- physical robot accuracy를 주장하지 않는다.
- `FK Position Error < 0.5 mm`는 이번 결과가 아니다.
"""


if __name__ == "__main__":
    write_evidence()
