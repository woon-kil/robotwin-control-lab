using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace Robotwin.Ur10e.Editor
{
    public sealed class Ur10eBaselineResult
    {
        public string Result;
        public int ConfigurationCount;
        public double GlobalMaxJointErrorRad;
        public string UnityVersion;
        public string FailureSummary;
        public int TimeoutCount;
    }

    public static class Ur10eJointBaselineRunner
    {
        const string PrefabPath = "Assets/Ur10eGolden/ur10e.prefab";
        const string ReportPath = "Assets/Ur10eGolden/import_report.json";
        const double RerunToleranceRadians = 1e-5;

        public static Ur10eBaselineResult Run()
        {
            var previousMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            try
            {
                return Measure();
            }
            finally
            {
                Physics.simulationMode = previousMode;
            }
        }

        static Ur10eBaselineResult Measure()
        {
            var evidenceDirectory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "evidence", "P-002"));
            Directory.CreateDirectory(evidenceDirectory);
            var importReport = File.ReadAllText(Path.Combine(Application.dataPath, "Ur10eGolden/import_report.json"));
            var invalidRejected = RejectsShortJointVector();
            var configurations = Configurations();
            var rows = new Row[configurations.Length];
            string[] mappingLines = null;
            int disabledColliders = -1;
            var rootWasImmovable = false;
            for (var index = 0; index < configurations.Length; index++)
            {
                rows[index] = MeasureConfiguration(configurations[index], index == 0, ref mappingLines, ref disabledColliders, ref rootWasImmovable);
            }

            var repeat = MeasureConfiguration(configurations[0], false, ref mappingLines, ref disabledColliders, ref rootWasImmovable);
            var rerunError = MaxAbsDiff(rows[0].Actual, repeat.Actual);
            var globalMax = 0.0;
            var failed = new StringBuilder();
            if (!invalidRejected)
            {
                failed.Append("invalid joint count was not rejected\n");
            }

            if (rerunError > RerunToleranceRadians)
            {
                failed.Append("A-zero rerun differs by ");
                failed.Append(rerunError.ToString("G17", CultureInfo.InvariantCulture));
                failed.Append('\n');
            }

            for (var index = 0; index < rows.Length; index++)
            {
                if (rows[index].MaxAbs > globalMax)
                {
                    globalMax = rows[index].MaxAbs;
                }

                if (rows[index].Result != "PASS")
                {
                    failed.Append(rows[index].Id);
                    failed.Append(' ');
                    failed.Append(rows[index].Result);
                    failed.Append(" max_abs_error_rad=");
                    failed.Append(rows[index].MaxAbs.ToString("G17", CultureInfo.InvariantCulture));
                    failed.Append('\n');
                }
            }

            var result = failed.Length == 0 ? "PASS" : "FAIL";
            var unityVersion = Application.unityVersion;
            var timeouts = 0;
            for (var index = 0; index < rows.Length; index++)
            {
                if (rows[index].TimedOut)
                {
                    timeouts++;
                }
            }

            var evidenceFiles = new[]
            {
                Path.Combine(evidenceDirectory, "joint_results.json"),
                Path.Combine(evidenceDirectory, "summary.md"),
                Path.Combine(evidenceDirectory, "environment.md"),
                Path.Combine(evidenceDirectory, "test_results.txt"),
            };
            WriteJointResults(evidenceFiles[0], rows, globalMax, result, rerunError, invalidRejected, disabledColliders, rootWasImmovable, mappingLines);
            WriteSummary(evidenceFiles[1], rows, globalMax, result, unityVersion, importReport, failed.ToString());
            WriteEnvironment(evidenceFiles[2], unityVersion, importReport, disabledColliders, rootWasImmovable);
            WriteTestLog(evidenceFiles[3], rows, globalMax, result, rerunError, invalidRejected);
            var attempt2 = Path.Combine(evidenceDirectory, "attempt2");
            Directory.CreateDirectory(attempt2);
            for (var index = 0; index < evidenceFiles.Length; index++)
            {
                File.Copy(evidenceFiles[index], Path.Combine(attempt2, Path.GetFileName(evidenceFiles[index])), true);
            }

            return new Ur10eBaselineResult
            {
                Result = result,
                ConfigurationCount = rows.Length,
                GlobalMaxJointErrorRad = globalMax,
                UnityVersion = unityVersion,
                FailureSummary = failed.ToString(),
                TimeoutCount = timeouts,
            };
        }

        static bool RejectsShortJointVector()
        {
            var instance = InstantiateRobot();
            try
            {
                var adapter = Ur10eJointAdapter.Bind(instance);
                var before = adapter.GetJointPositionsRadians();
                var rejected = false;
                try
                {
                    adapter.SetJointTargetsRadians(new[] { 0.1, 0.1, 0.1, 0.1, 0.1 });
                }
                catch (Ur10eAdapterException exception)
                {
                    rejected = exception.Code == "JOINT_COUNT";
                }

                var after = adapter.GetJointPositionsRadians();
                return rejected && MaxAbsDiff(before, after) == 0.0;
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(instance);
            }
        }

        static Row MeasureConfiguration(JointConfiguration configuration, bool captureSetup, ref string[] mappingLines, ref int disabledColliders, ref bool rootWasImmovable)
        {
            var instance = InstantiateRobot();
            try
            {
                Physics.SyncTransforms();
                var adapter = Ur10eJointAdapter.Bind(instance);
                if (captureSetup)
                {
                    mappingLines = new string[adapter.Joints.Length];
                    for (var index = 0; index < adapter.Joints.Length; index++)
                    {
                        var joint = adapter.Joints[index];
                        mappingLines[index] = joint.CanonicalName + "\t" + joint.UnityPath + "\t" + joint.JointType;
                    }
                }

                disabledColliders = adapter.DisableColliders();
                rootWasImmovable = adapter.EnsureRootImmovable();
                adapter.ApplyProjectSolverIterations();
                adapter.SetJointTargetsRadians(configuration.Joints);
                var step = Time.fixedDeltaTime;
                var steps = 0;
                var stableFrames = 0;
                var previous = adapter.GetJointPositionsRadians();
                var actual = previous;
                var elapsed = 0.0;
                var settled = false;
                while (elapsed < Ur10eProjectTestParameters.TimeoutSeconds)
                {
                    Physics.Simulate(step);
                    steps++;
                    elapsed += step;
                    actual = adapter.GetJointPositionsRadians();
                    var velocity = adapter.MaxAbsJointVelocityRadiansPerSecond();
                    var delta = MaxAbsDiff(actual, previous);
                    if (velocity < Ur10eProjectTestParameters.VelocityEpsilonRadPerSec
                        && delta < Ur10eProjectTestParameters.PositionDeltaEpsilonRad)
                    {
                        stableFrames++;
                    }
                    else
                    {
                        stableFrames = 0;
                    }

                    previous = actual;
                    if (stableFrames >= Ur10eProjectTestParameters.SettledFramesRequired)
                    {
                        settled = true;
                        break;
                    }
                }

                var error = new double[actual.Length];
                var maxAbs = 0.0;
                var finite = true;
                for (var index = 0; index < actual.Length; index++)
                {
                    error[index] = actual[index] - configuration.Joints[index];
                    var abs = Math.Abs(error[index]);
                    if (abs > maxAbs)
                    {
                        maxAbs = abs;
                    }

                    if (double.IsNaN(actual[index]) || double.IsInfinity(actual[index]))
                    {
                        finite = false;
                    }
                }

                var pose = adapter.ReadTool0Pose();
                if (!IsFinite(pose.PositionMeters) || !IsFinite(pose.Rotation))
                {
                    finite = false;
                }

                var timedOut = !settled;
                var passed = settled && finite && maxAbs < Ur10eProjectTestParameters.EngineeringTargetRadians;
                return new Row
                {
                    Id = configuration.Id,
                    Target = configuration.Joints,
                    Actual = actual,
                    Error = error,
                    MaxAbs = maxAbs,
                    Steps = steps,
                    SettlingTime = steps * step,
                    TimedOut = timedOut,
                    Pose = pose,
                    Result = passed ? "PASS" : "FAIL",
                };
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(instance);
            }
        }

        static GameObject InstantiateRobot()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            if (prefab == null)
            {
                throw new Ur10eAdapterException("INVALID_MODEL", "prefab missing at " + PrefabPath);
            }

            var instance = PrefabUtility.InstantiatePrefab(prefab) as GameObject;
            if (instance == null)
            {
                throw new Ur10eAdapterException("INVALID_MODEL", "prefab instantiate failed");
            }

            return instance;
        }

        static JointConfiguration[] Configurations()
        {
            return new[]
            {
                new JointConfiguration("A-zero", new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }),
                new JointConfiguration("B-positive-1", new[] { 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 }),
                new JointConfiguration("C-mixed-1", new[] { 0.5, -0.4, 0.3, -0.2, 0.6, -0.7 }),
                new JointConfiguration("E-fixed-1", new[] { 0.1, -0.2, 0.3, -0.4, 0.5, -0.6 }),
                new JointConfiguration("E-fixed-3", new[] { 0.01, 0.02, -0.03, 0.04, -0.05, 0.06 }),
            };
        }

        static void WriteJointResults(string path, Row[] rows, double globalMax, string result, double rerunError, bool invalidRejected, int disabledColliders, bool rootWasImmovable, string[] mappingLines)
        {
            var builder = new StringBuilder();
            builder.Append("{\n");
            AppendString(builder, "gate", "P-002", 1, true);
            AppendString(builder, "source_repository", "UniversalRobots/Universal_Robots_ROS2_Description", 1, true);
            AppendString(builder, "source_commit", "7ce98d8c3512b9d0239da93b75de68ea64f61bd6", 1, true);
            AppendString(builder, "robot_type", "ur10e", 1, true);
            AppendString(builder, "source_manifest", "robot_description/SOURCE_MANIFEST.md", 1, true);
            AppendString(builder, "unity_model", PrefabPath, 1, true);
            AppendString(builder, "import_report", ReportPath, 1, true);
            AppendBool(builder, "invalid_joint_count_rejected", invalidRejected, 1, true);
            AppendNumber(builder, "deterministic_rerun_max_abs_rad", rerunError, 1, true);
            AppendNumber(builder, "colliders_disabled", disabledColliders, 1, true);
            AppendBool(builder, "root_was_immovable_before_test", rootWasImmovable, 1, true);
            AppendNumber(builder, "solver_iterations", Ur10eProjectTestParameters.SolverIterations, 1, true);
            AppendNumber(builder, "solver_velocity_iterations", Ur10eProjectTestParameters.SolverVelocityIterations, 1, true);
            AppendNumber(builder, "global_max_joint_error_rad", globalMax, 1, true);
            AppendNumber(builder, "global_max_joint_error_deg", globalMax * Ur10eJointAdapter.RadiansToDegrees, 1, true);
            AppendNumber(builder, "engineering_target_deg", Ur10eProjectTestParameters.EngineeringTargetDegrees, 1, true);
            AppendString(builder, "result", result, 1, true);
            builder.Append("  \"canonical_joint_order\": [");
            for (var index = 0; index < Ur10eJointAdapter.CanonicalOrder.Length; index++)
            {
                if (index > 0)
                {
                    builder.Append(", ");
                }

                builder.Append('"');
                builder.Append(Ur10eJointAdapter.CanonicalOrder[index]);
                builder.Append('"');
            }

            builder.Append("],\n");
            builder.Append("  \"mapping\": [\n");
            for (var index = 0; index < mappingLines.Length; index++)
            {
                var parts = mappingLines[index].Split('\t');
                if (index > 0)
                {
                    builder.Append(",\n");
                }

                builder.Append("    {\"canonical_name\": \"");
                builder.Append(parts[0]);
                builder.Append("\", \"unity_path\": \"");
                builder.Append(parts[1]);
                builder.Append("\", \"joint_type\": \"");
                builder.Append(parts[2]);
                builder.Append("\"}");
            }

            builder.Append("\n  ],\n");
            builder.Append("  \"configurations\": [\n");
            for (var index = 0; index < rows.Length; index++)
            {
                if (index > 0)
                {
                    builder.Append(",\n");
                }

                WriteRow(builder, rows[index]);
            }

            builder.Append("\n  ]\n}\n");
            File.WriteAllText(path, builder.ToString());
        }

        static void WriteRow(StringBuilder builder, Row row)
        {
            builder.Append("    {\n");
            AppendString(builder, "test_id", row.Id, 3, true);
            AppendArray(builder, "target_joint_rad", row.Target, 3, true);
            AppendArray(builder, "actual_joint_rad", row.Actual, 3, true);
            AppendArray(builder, "error_joint_rad", row.Error, 3, true);
            AppendNumber(builder, "max_abs_error_rad", row.MaxAbs, 3, true);
            AppendNumber(builder, "max_abs_error_deg", row.MaxAbs * Ur10eJointAdapter.RadiansToDegrees, 3, true);
            AppendNumber(builder, "settling_steps", row.Steps, 3, true);
            AppendNumber(builder, "settling_time", row.SettlingTime, 3, true);
            AppendNumber(builder, "timeout", Ur10eProjectTestParameters.TimeoutSeconds, 3, true);
            AppendBool(builder, "timed_out", row.TimedOut, 3, true);
            builder.Append("      \"tool0_pose\": {\n");
            AppendString(builder, "frame", row.Pose.Frame, 4, true);
            builder.Append("        \"position_m\": {\"x\": ");
            builder.Append(Num(row.Pose.PositionMeters.x));
            builder.Append(", \"y\": ");
            builder.Append(Num(row.Pose.PositionMeters.y));
            builder.Append(", \"z\": ");
            builder.Append(Num(row.Pose.PositionMeters.z));
            builder.Append("},\n");
            builder.Append("        \"quaternion_xyzw\": {\"x\": ");
            builder.Append(Num(row.Pose.Rotation.x));
            builder.Append(", \"y\": ");
            builder.Append(Num(row.Pose.Rotation.y));
            builder.Append(", \"z\": ");
            builder.Append(Num(row.Pose.Rotation.z));
            builder.Append(", \"w\": ");
            builder.Append(Num(row.Pose.Rotation.w));
            builder.Append("}\n      },\n");
            AppendString(builder, "result", row.Result, 3, false);
            builder.Append("    }");
        }

        static void WriteSummary(string path, Row[] rows, double globalMax, string result, string unityVersion, string importReport, string failures)
        {
            var degrees = globalMax * Ur10eJointAdapter.RadiansToDegrees;
            var builder = new StringBuilder();
            builder.Append("# P-002 Unity UR10e joint control\n\n```text\n");
            builder.Append("Gate\nP-002\n\nResult\n");
            builder.Append(result);
            builder.Append("\n\nRobot\nUR10e Golden Reference\n\nSource commit\n7ce98d8c3512b9d0239da93b75de68ea64f61bd6\n\nConfigurations\n");
            builder.Append(rows.Length.ToString(CultureInfo.InvariantCulture));
            builder.Append("\n\nGlobal max joint error\n");
            builder.Append(globalMax.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append(" rad\n");
            builder.Append(degrees.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append(" deg\n\nEngineering target\n< 0.1 deg\n\nUnity version\n");
            builder.Append(unityVersion);
            builder.Append("\n\nURDF Importer\n");
            builder.Append(JsonString(importReport, "urdf_importer_package_version"));
            builder.Append("\n");
            builder.Append(JsonString(importReport, "urdf_importer_hash"));
            builder.Append("\n```\n\n");
            builder.Append("측정값과 Engineering Target은 별개다. 이 숫자는 Unity articulation feedback이다. Python FK와 맞다는 판정은 아니다.\n\n");
            builder.Append("## Excluded P-001B configurations\n\n");
            builder.Append("D-near-upper와 D-near-lower는 ±360 deg 또는 ±180 deg 한계의 1% 안쪽이다. 이번 Gate의 drive는 PROJECT TEST PARAMETER라 limit 근처의 winding을 넣지 않았다.\n\n");
            builder.Append("B-positive-2, C-mixed-2, E-fixed-2, E-fixed-4는 같은 이유의 더 큰 이동이다. zero, positive, mixed, fixed, additional은 A-zero, B-positive-1, C-mixed-1, E-fixed-1, E-fixed-3으로 포함했다.\n\n");
            builder.Append("## Known limitations\n\n");
            builder.Append("- official nominal UR10e model 기반\n");
            builder.Append("- robot-specific calibration이 아님\n");
            builder.Append("- 실제 UR10e 측정값과 비교하지 않음\n");
            builder.Append("- Isaac Sim과 비교하지 않음\n");
            builder.Append("- Python FK와 최종 정확도 비교를 하지 않음\n");
            builder.Append("- physical robot accuracy를 주장하지 않음\n");
            builder.Append("- tool0 pose는 unity_world이며 canonical base_link로 변환하지 않음\n");
            builder.Append("- drive stiffness/damping, collider disable, solver iterations는 PROJECT TEST PARAMETER\n");
            builder.Append("- 이 측정은 실제 UR10e controller를 재현하지 않음\n");
            var evidenceDirectory = Path.GetDirectoryName(path);
            var attempt1 = Path.Combine(evidenceDirectory, "attempt1", "joint_results.json");
            var attempt2A = Path.Combine(evidenceDirectory, "attempt2a", "joint_results.json");
            builder.Append("\n## Attempt history\n\n```text\n");
            AppendAttempt(builder, "Attempt 1", attempt1);
            AppendAttempt(builder, "Attempt 2A", attempt2A);
            builder.Append("Attempt 2 final\n");
            builder.Append(result);
            builder.Append('\n');
            builder.Append(globalMax.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append(" rad\n");
            builder.Append(degrees.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append(" deg\n```\n\n");
            builder.Append("Attempt 1은 stiffness 100000, damping 10000, timeout 5s, velocity epsilon 0.01 rad/s, solver iterations 6이다. Attempt 2A는 stiffness 1000000, damping 2000, timeout 8s, velocity epsilon 0.001 rad/s, solver iterations 6이다. Attempt 2 final은 Attempt 2A와 같은 drive, timeout, gravity ON이고 solver position iterations만 24다.\n");
            builder.Append("\n## Cause\n\n");
            builder.Append("A-zero shoulder_lift는 gravity ON에서 약 0.4초 뒤에 위치가 멈추지만 오차는 0.150 deg에 남았다. 그때 drive force는 약 156 Nm이고 URDF effort 330 Nm에 닿지 않았다. stiffness 1000000/2000000/3000000과 damping 500/2000/8000은 그 바닥을 0.134 deg 아래로 내리지 못했다. solver position iterations 6, 12, 24에서는 같은 A-zero 오차가 0.150 deg, 0.053 deg, 0.016 deg였다. iterations 24에서 reported joint velocity도 0.001 rad/s 아래였다. timestep 0.005는 오차를 0.018 deg까지 줄였지만 reported velocity가 0.001 rad/s보다 커서 settle 조건을 통과하지 못했다. velocity iterations 4는 오차를 줄이지 못했다. force limit, 무한 stiffness, gravity OFF는 최종 조건으로 쓰지 않았다. 상세 숫자는 evidence/P-002/attempt2/diagnostics.md다.\n");
            if (failures.Length > 0)
            {
                builder.Append("\n## Failures\n\n```text\n");
                builder.Append(failures);
                builder.Append("```\n");
            }

            File.WriteAllText(path, builder.ToString());
        }

        static void WriteEnvironment(string path, string unityVersion, string importReport, int disabledColliders, bool rootWasImmovable)
        {
            var builder = new StringBuilder();
            builder.Append("# P-002 environment\n\n```text\n");
            builder.Append("OS\n");
            builder.Append(SystemInfo.operatingSystem);
            builder.Append("\n\nUnity Editor version\n");
            builder.Append(unityVersion);
            builder.Append("\n\nURDF Importer version\n");
            builder.Append(JsonString(importReport, "urdf_importer_package_version"));
            builder.Append("\n");
            builder.Append(JsonString(importReport, "urdf_importer_hash"));
            builder.Append("\n\nImport method\n");
            builder.Append("Unity.Robotics.UrdfImporter.UrdfRobotExtensions.Create\n");
            builder.Append("chosenAxis ");
            builder.Append(JsonString(importReport, "chosen_axis"));
            builder.Append("\nconvexMethod ");
            builder.Append(JsonString(importReport, "convex_method"));
            builder.Append("\n\nSource model\nrobot_description/generated/7ce98d8c3512b9d0239da93b75de68ea64f61bd6/ur10e.urdf\n\nSource commit\n7ce98d8c3512b9d0239da93b75de68ea64f61bd6\n\n");
            builder.Append("source repository\nUniversalRobots/Universal_Robots_ROS2_Description\nrobot\nur10e\nsource manifest\nrobot_description/SOURCE_MANIFEST.md\n");
            builder.Append("physics timestep\n");
            builder.Append(Time.fixedDeltaTime.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append("\ngravity\n");
            builder.Append(Physics.gravity.x.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append(' ');
            builder.Append(Physics.gravity.y.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append(' ');
            builder.Append(Physics.gravity.z.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append("\n\ntest runner\n");
            builder.Append(TestCommand());
            builder.Append("\n```\n\n");
            builder.Append("## Unit boundary\n\n");
            builder.Append("Canonical joint command와 feedback은 radian이다. `ArticulationDrive.target`은 angular motion에서 degree다. 변환은 `Ur10eJointAdapter.SetJointTargetsRadians`에서 `radian * 180 / pi`로만 한다. `ArticulationBody.jointPosition[0]`은 radian이라 feedback에 추가 변환을 하지 않는다.\n\n");
            builder.Append("근거: https://docs.unity3d.com/2022.3/Documentation/ScriptReference/ArticulationDrive-target.html\n");
            builder.Append("근거: https://docs.unity3d.com/2022.3/Documentation/ScriptReference/ArticulationBody-jointPosition.html\n\n");
            builder.Append("## Coordinate boundary\n\n");
            builder.Append("어댑터는 TCP pose를 canonical base_link로 다시 바꾸지 않는다. 기록하는 tool0은 Unity world pose다. 위치 단위는 meter, quaternion은 Unity `(x, y, z, w)`다.\n\n");
            builder.Append("링크 배치와 관절 축은 URDF Importer가 import 때 적용한다. 벡터 `Ros2Unity`은 `(-rosY, rosZ, rosX)`다. quaternion은 `(rosY, -rosZ, -rosX, rosW)`다. origin Euler degree는 `(pitch, -yaw, -roll)`이다. revolute twist는 parent anchor의 X축이고, importer가 `SetFromToRotation(+X, -Ros2Unity(axis))`로 anchor를 돌린다.\n\n");
            builder.Append("`chosenAxis=zAxis`는 importer의 mesh up-axis 설정이다. source mesh의 up axis가 Z이기 때문에 그 값을 넘겼다. 이 설정은 mesh visual/collision 보정이며 canonical joint radian을 바꾸지 않는다.\n\n");
            builder.Append("## PROJECT TEST PARAMETER\n\n```text\n");
            builder.Append("drive stiffness\n");
            builder.Append(Ur10eProjectTestParameters.DriveStiffness.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append("\ndrive damping\n");
            builder.Append(Ur10eProjectTestParameters.DriveDamping.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append("\nsolver iterations\n");
            builder.Append(Ur10eProjectTestParameters.SolverIterations.ToString(CultureInfo.InvariantCulture));
            builder.Append("\nsolver velocity iterations\n");
            builder.Append(Ur10eProjectTestParameters.SolverVelocityIterations.ToString(CultureInfo.InvariantCulture));
            builder.Append("\nforce limit\nimported URDF effort, unchanged\ncolliders disabled\n");
            builder.Append(disabledColliders.ToString(CultureInfo.InvariantCulture));
            builder.Append("\nroot was immovable before test\n");
            builder.Append(rootWasImmovable ? "true" : "false");
            builder.Append("\nsettled frames\n");
            builder.Append(Ur10eProjectTestParameters.SettledFramesRequired.ToString(CultureInfo.InvariantCulture));
            builder.Append("\nvelocity epsilon rad/s\n");
            builder.Append(Ur10eProjectTestParameters.VelocityEpsilonRadPerSec.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append("\nposition delta epsilon rad\n");
            builder.Append(Ur10eProjectTestParameters.PositionDeltaEpsilonRad.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append("\ntimeout seconds\n");
            builder.Append(Ur10eProjectTestParameters.TimeoutSeconds.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append("\n```\n\n");
            builder.Append("Importer가 남긴 stiffness/damping은 import_report.json의 imported_stiffness, imported_damping이다. runner가 그 값을 산업 사양으로 쓰지 않고, 측정 중에만 위 PROJECT TEST PARAMETER를 drive에 넣는다. force limit은 바꾸지 않는다.\n");
            File.WriteAllText(path, builder.ToString());
        }

        static void WriteTestLog(string path, Row[] rows, double globalMax, string result, double rerunError, bool invalidRejected)
        {
            var builder = new StringBuilder();
            builder.Append(TestCommand());
            builder.Append("\n\nresult ");
            builder.Append(result);
            builder.Append("\nglobal_max_joint_error_rad ");
            builder.Append(globalMax.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append("\ninvalid_joint_count_rejected ");
            builder.Append(invalidRejected ? "true" : "false");
            builder.Append("\ndeterministic_rerun_max_abs_rad ");
            builder.Append(rerunError.ToString("G17", CultureInfo.InvariantCulture));
            builder.Append('\n');
            for (var index = 0; index < rows.Length; index++)
            {
                builder.Append(rows[index].Id);
                builder.Append(' ');
                builder.Append(rows[index].Result);
                builder.Append(" steps=");
                builder.Append(rows[index].Steps.ToString(CultureInfo.InvariantCulture));
                builder.Append(" max_abs_error_rad=");
                builder.Append(rows[index].MaxAbs.ToString("G17", CultureInfo.InvariantCulture));
                builder.Append(" timed_out=");
                builder.Append(rows[index].TimedOut ? "true" : "false");
                builder.Append('\n');
            }

            File.WriteAllText(path, builder.ToString());
        }

        static string TestCommand()
        {
            return "/Applications/Unity/Hub/Editor/2022.3.62f2/Unity.app/Contents/MacOS/Unity -batchmode -nographics -projectPath unity/UR10eJointBaseline -runTests -testPlatform EditMode -testCategory Ur10eJointBaseline -testResults evidence/P-002/unity-test-results.xml";
        }

        static void AppendAttempt(StringBuilder builder, string label, string path)
        {
            builder.Append(label);
            builder.Append('\n');
            if (!File.Exists(path))
            {
                builder.Append("missing\n");
                return;
            }

            var json = File.ReadAllText(path);
            builder.Append(JsonString(json, "result"));
            builder.Append('\n');
            builder.Append(JsonNumber(json, "global_max_joint_error_rad"));
            builder.Append(" rad\n");
        }

        static string JsonString(string json, string field)
        {
            var token = "\"" + field + "\"";
            var index = json.IndexOf(token, StringComparison.Ordinal);
            var colon = json.IndexOf(':', index + token.Length);
            var start = json.IndexOf('"', colon + 1);
            var end = json.IndexOf('"', start + 1);
            return json.Substring(start + 1, end - start - 1);
        }

        static string JsonNumber(string json, string field)
        {
            var token = "\"" + field + "\"";
            var index = json.IndexOf(token, StringComparison.Ordinal);
            var colon = json.IndexOf(':', index + token.Length);
            var start = colon + 1;
            while (start < json.Length && char.IsWhiteSpace(json[start]))
            {
                start++;
            }

            var end = start;
            while (end < json.Length && json[end] != ',' && json[end] != '\n' && json[end] != '}')
            {
                end++;
            }

            return json.Substring(start, end - start).Trim();
        }

        static double MaxAbsDiff(double[] left, double[] right)
        {
            var max = 0.0;
            for (var index = 0; index < left.Length; index++)
            {
                var abs = Math.Abs(left[index] - right[index]);
                if (abs > max)
                {
                    max = abs;
                }
            }

            return max;
        }

        static bool IsFinite(Vector3 value)
        {
            return !(float.IsNaN(value.x) || float.IsNaN(value.y) || float.IsNaN(value.z)
                || float.IsInfinity(value.x) || float.IsInfinity(value.y) || float.IsInfinity(value.z));
        }

        static bool IsFinite(Quaternion value)
        {
            return !(float.IsNaN(value.x) || float.IsNaN(value.y) || float.IsNaN(value.z) || float.IsNaN(value.w)
                || float.IsInfinity(value.x) || float.IsInfinity(value.y) || float.IsInfinity(value.z) || float.IsInfinity(value.w));
        }

        static void AppendString(StringBuilder builder, string name, string value, int indent, bool comma)
        {
            builder.Append(' ', indent * 2);
            builder.Append('"');
            builder.Append(name);
            builder.Append("\": \"");
            builder.Append(value);
            builder.Append('"');
            if (comma)
            {
                builder.Append(',');
            }

            builder.Append('\n');
        }

        static void AppendNumber(StringBuilder builder, string name, double value, int indent, bool comma)
        {
            builder.Append(' ', indent * 2);
            builder.Append('"');
            builder.Append(name);
            builder.Append("\": ");
            builder.Append(Num(value));
            if (comma)
            {
                builder.Append(',');
            }

            builder.Append('\n');
        }

        static void AppendBool(StringBuilder builder, string name, bool value, int indent, bool comma)
        {
            builder.Append(' ', indent * 2);
            builder.Append('"');
            builder.Append(name);
            builder.Append("\": ");
            builder.Append(value ? "true" : "false");
            if (comma)
            {
                builder.Append(',');
            }

            builder.Append('\n');
        }

        static void AppendArray(StringBuilder builder, string name, double[] values, int indent, bool comma)
        {
            builder.Append(' ', indent * 2);
            builder.Append('"');
            builder.Append(name);
            builder.Append("\": [");
            for (var index = 0; index < values.Length; index++)
            {
                if (index > 0)
                {
                    builder.Append(", ");
                }

                builder.Append(Num(values[index]));
            }

            builder.Append(']');
            if (comma)
            {
                builder.Append(',');
            }

            builder.Append('\n');
        }

        static string Num(double value)
        {
            return value.ToString("G17", CultureInfo.InvariantCulture);
        }

        sealed class Row
        {
            public string Id;
            public double[] Target;
            public double[] Actual;
            public double[] Error;
            public double MaxAbs;
            public int Steps;
            public double SettlingTime;
            public bool TimedOut;
            public Ur10eTool0Pose Pose;
            public string Result;
        }

        sealed class JointConfiguration
        {
            public readonly string Id;
            public readonly double[] Joints;

            public JointConfiguration(string id, double[] joints)
            {
                Id = id;
                Joints = joints;
            }
        }
    }
}
