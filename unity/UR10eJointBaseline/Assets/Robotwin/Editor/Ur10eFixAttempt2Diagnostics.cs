using System;
using System.Globalization;
using System.IO;
using System.Text;
using Robotwin.Ur10e;
using UnityEditor;
using UnityEngine;

namespace Robotwin.Ur10e.Editor
{
    public static class Ur10eFixAttempt2Diagnostics
    {
        public static void Run()
        {
            var previousMode = Physics.simulationMode;
            var previousGravity = Physics.gravity;
            Physics.simulationMode = SimulationMode.Script;
            var exitCode = 0;
            try
            {
                var directory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "evidence", "P-002", "attempt2"));
                Directory.CreateDirectory(directory);
                File.WriteAllText(Path.Combine(directory, "diagnostic_raw.json"), BuildReport());
                File.WriteAllText(Path.Combine(directory, "shoulder_lift_trace.json"), BuildTrace());
                Debug.Log("P-002 fix attempt 2 diagnostics written");
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                exitCode = 1;
            }
            finally
            {
                Physics.simulationMode = previousMode;
                Physics.gravity = previousGravity;
            }

            EditorApplication.Exit(exitCode);
        }

        public static void RunSolverProbe()
        {
            var previousMode = Physics.simulationMode;
            var previousGravity = Physics.gravity;
            var previousStep = Time.fixedDeltaTime;
            var previousIterations = Physics.defaultSolverIterations;
            var previousVelocityIterations = Physics.defaultSolverVelocityIterations;
            Physics.simulationMode = SimulationMode.Script;
            var exitCode = 0;
            try
            {
                var directory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "evidence", "P-002", "attempt2"));
                Directory.CreateDirectory(directory);
                File.WriteAllText(Path.Combine(directory, "solver_probe.json"), BuildSolverProbe());
                Debug.Log("P-002 solver probe written");
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                exitCode = 1;
            }
            finally
            {
                Time.fixedDeltaTime = previousStep;
                Physics.defaultSolverIterations = previousIterations;
                Physics.defaultSolverVelocityIterations = previousVelocityIterations;
                Physics.simulationMode = previousMode;
                Physics.gravity = previousGravity;
            }

            EditorApplication.Exit(exitCode);
        }

        static string BuildSolverProbe()
        {
            var builder = new StringBuilder();
            builder.Append("{\"marker\":\"DIAGNOSTIC ONLY\",\"configuration\":\"A-zero\",\"stiffness\":1000000,\"damping\":2000,\"rows\":[\n");
            var first = true;
            var cases = new[]
            {
                new[] { 0.02f, 6f, 1f },
                new[] { 0.01f, 6f, 1f },
                new[] { 0.005f, 6f, 1f },
                new[] { 0.02f, 12f, 1f },
                new[] { 0.02f, 24f, 1f },
                new[] { 0.02f, 6f, 4f },
                new[] { 0.01f, 12f, 4f },
            };
            foreach (var item in cases)
            {
                Time.fixedDeltaTime = item[0];
                Physics.defaultSolverIterations = (int)item[1];
                Physics.defaultSolverVelocityIterations = (int)item[2];
                var steps = (int)Math.Round(2.0 / item[0]);
                var instance = PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Ur10eGolden/ur10e.prefab")) as GameObject;
                try
                {
                    Physics.SyncTransforms();
                    var adapter = Ur10eJointAdapter.Bind(instance);
                    foreach (var body in instance.GetComponentsInChildren<ArticulationBody>())
                    {
                        body.solverIterations = (int)item[1];
                        body.solverVelocityIterations = (int)item[2];
                    }

                    adapter.DisableColliders();
                    adapter.SetRootImmovable(true);
                    var targets = new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
                    adapter.SetJointTargetsRadians(targets);
                    for (var step = 0; step < steps; step++)
                    {
                        Physics.Simulate(item[0]);
                    }

                    var sample = adapter.SampleJoints(targets)[1];
                    if (!first)
                    {
                        builder.Append(",\n");
                    }

                    first = false;
                    builder.Append("{\"dt\":");
                    builder.Append(Num(item[0]));
                    builder.Append(",\"solver_iterations\":");
                    builder.Append(((int)item[1]).ToString(CultureInfo.InvariantCulture));
                    builder.Append(",\"velocity_iterations\":");
                    builder.Append(((int)item[2]).ToString(CultureInfo.InvariantCulture));
                    builder.Append(",\"steps\":");
                    builder.Append(steps.ToString(CultureInfo.InvariantCulture));
                    builder.Append(",\"error_deg\":");
                    builder.Append(Num((sample.ActualRadians - sample.TargetRadians) * Ur10eJointAdapter.RadiansToDegrees));
                    builder.Append(",\"reported_velocity_rad_s\":");
                    builder.Append(Num(sample.VelocityRadiansPerSecond));
                    builder.Append(",\"drive_force_nm\":");
                    builder.Append(sample.DriveForceReadable ? Num(sample.DriveForceNewtonMeters) : "null");
                    builder.Append('}');
                }
                finally
                {
                    if (instance != null)
                    {
                        UnityEngine.Object.DestroyImmediate(instance);
                    }
                }
            }

            builder.Append("\n]}");
            return builder.ToString();
        }

        public static void RunSweep()
        {
            var previousMode = Physics.simulationMode;
            var previousGravity = Physics.gravity;
            Physics.simulationMode = SimulationMode.Script;
            var exitCode = 0;
            try
            {
                var directory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "evidence", "P-002", "attempt2"));
                Directory.CreateDirectory(directory);
                File.WriteAllText(Path.Combine(directory, "stiffness_sweep.json"), BuildSweep());
                Debug.Log("P-002 stiffness sweep written");
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                exitCode = 1;
            }
            finally
            {
                Physics.simulationMode = previousMode;
                Physics.gravity = previousGravity;
            }

            EditorApplication.Exit(exitCode);
        }

        static string BuildSweep()
        {
            var stiffnesses = new[] { 1000000f, 2000000f, 3000000f };
            var dampings = new[] { 500f, 2000f, 8000f };
            var targets = new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            var builder = new StringBuilder();
            builder.Append("{\"marker\":\"DIAGNOSTIC ONLY\",\"configuration\":\"A-zero\",\"gravity_on\":true,\"root_fixed\":true,\"colliders_disabled\":true,\"reason\":\"measured A-zero shoulder_lift equilibrium is 0.150046 deg at stiffness 1000000 and damping 2000; candidates scale that operating point\",\"rows\":[\n");
            var first = true;
            foreach (var stiffness in stiffnesses)
            {
                foreach (var damping in dampings)
                {
                    var instance = PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Ur10eGolden/ur10e.prefab")) as GameObject;
                    try
                    {
                        Physics.SyncTransforms();
                        var adapter = Ur10eJointAdapter.Bind(instance);
                        adapter.DisableColliders();
                        adapter.SetRootImmovable(true);
                        adapter.SetJointTargetsRadians(targets, stiffness, damping);
                        var previousError = 0.0;
                        var stable = 0;
                        var settledStep = -1;
                        for (var step = 1; step <= 400; step++)
                        {
                            Physics.Simulate(Time.fixedDeltaTime);
                            var sample = adapter.SampleJoints(targets)[1];
                            var error = sample.ActualRadians - sample.TargetRadians;
                            var delta = Math.Abs(error - previousError);
                            previousError = error;
                            if (delta < Ur10eProjectTestParameters.PositionDeltaEpsilonRad)
                            {
                                stable++;
                                if (stable >= Ur10eProjectTestParameters.SettledFramesRequired && settledStep < 0)
                                {
                                    settledStep = step;
                                }
                            }
                            else
                            {
                                stable = 0;
                            }
                        }

                        var finalSample = adapter.SampleJoints(targets)[1];
                        if (!first)
                        {
                            builder.Append(",\n");
                        }

                        first = false;
                        builder.Append("{\"stiffness\":");
                        builder.Append(Num(stiffness));
                        builder.Append(",\"damping\":");
                        builder.Append(Num(damping));
                        builder.Append(",\"error_deg\":");
                        builder.Append(Num((finalSample.ActualRadians - finalSample.TargetRadians) * Ur10eJointAdapter.RadiansToDegrees));
                        builder.Append(",\"reported_velocity_rad_s\":");
                        builder.Append(Num(finalSample.VelocityRadiansPerSecond));
                        builder.Append(",\"drive_force_nm\":");
                        builder.Append(finalSample.DriveForceReadable ? Num(finalSample.DriveForceNewtonMeters) : "null");
                        builder.Append(",\"force_limit\":");
                        builder.Append(Num(finalSample.ForceLimit));
                        builder.Append(",\"position_settled_step\":");
                        builder.Append(settledStep.ToString(CultureInfo.InvariantCulture));
                        builder.Append('}');
                    }
                    finally
                    {
                        if (instance != null)
                        {
                            UnityEngine.Object.DestroyImmediate(instance);
                        }
                    }
                }
            }

            builder.Append("\n]}");
            return builder.ToString();
        }

        static string BuildTrace()
        {
            var builder = new StringBuilder();
            builder.Append("{\"case_id\":\"G-ON-A-zero-trace\",\"marker\":\"DIAGNOSTIC ONLY\",\"samples\":[\n");
            var instance = PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Ur10eGolden/ur10e.prefab")) as GameObject;
            try
            {
                Physics.SyncTransforms();
                var adapter = Ur10eJointAdapter.Bind(instance);
                adapter.DisableColliders();
                adapter.SetRootImmovable(true);
                var targets = new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
                adapter.SetJointTargetsRadians(targets);
                var step = (double)Time.fixedDeltaTime;
                var first = true;
                for (var stepIndex = 0; stepIndex <= 400; stepIndex++)
                {
                    if (stepIndex > 0)
                    {
                        Physics.Simulate(Time.fixedDeltaTime);
                    }

                    var sample = adapter.SampleJoints(targets)[1];
                    if (!first)
                    {
                        builder.Append(",\n");
                    }

                    first = false;
                    builder.Append("{\"step\":");
                    builder.Append(stepIndex.ToString(CultureInfo.InvariantCulture));
                    builder.Append(",\"time\":");
                    builder.Append(Num(stepIndex * step));
                    builder.Append(",\"error_deg\":");
                    builder.Append(Num((sample.ActualRadians - sample.TargetRadians) * Ur10eJointAdapter.RadiansToDegrees));
                    builder.Append(",\"velocity_rad_s\":");
                    builder.Append(Num(sample.VelocityRadiansPerSecond));
                    builder.Append(",\"drive_force_nm\":");
                    builder.Append(sample.DriveForceReadable ? Num(sample.DriveForceNewtonMeters) : "null");
                    builder.Append('}');
                }
            }
            finally
            {
                if (instance != null)
                {
                    UnityEngine.Object.DestroyImmediate(instance);
                }
            }

            builder.Append("\n]}");
            return builder.ToString();
        }

        static string BuildReport()
        {
            var builder = new StringBuilder();
            builder.Append("{\n");
            builder.Append("  \"gate\": \"P-002 Fix Attempt 2\",\n");
            builder.Append("  \"stage\": \"root-cause before parameter change\",\n");
            builder.Append("  \"fixed_timestep\": ");
            builder.Append(Num(Time.fixedDeltaTime));
            builder.Append(",\n  \"drive_stiffness\": ");
            builder.Append(Num(Ur10eProjectTestParameters.DriveStiffness));
            builder.Append(",\n  \"drive_damping\": ");
            builder.Append(Num(Ur10eProjectTestParameters.DriveDamping));
            builder.Append(",\n  \"timeout_seconds\": ");
            builder.Append(Num(Ur10eProjectTestParameters.TimeoutSeconds));
            builder.Append(",\n  \"velocity_epsilon_rad_s\": ");
            builder.Append(Num(Ur10eProjectTestParameters.VelocityEpsilonRadPerSec));
            builder.Append(",\n  \"position_delta_epsilon_rad\": ");
            builder.Append(Num(Ur10eProjectTestParameters.PositionDeltaEpsilonRad));
            builder.Append(",\n  \"settled_frames\": ");
            builder.Append(Ur10eProjectTestParameters.SettledFramesRequired.ToString(CultureInfo.InvariantCulture));
            builder.Append(",\n  \"cases\": [\n");
            AppendCase(builder, "G-ON", false, "current failing condition", Physics.gravity, true, true, false);
            builder.Append(",\n");
            AppendCase(builder, "G-OFF", true, "gravity contribution", Vector3.zero, true, true, false);
            builder.Append(",\n");
            AppendCase(builder, "A-zero-root-movable", true, "base fixation", Physics.gravity, false, true, true);
            builder.Append(",\n");
            AppendCase(builder, "A-zero-colliders-enabled", true, "collider contribution", Physics.gravity, true, false, true);
            builder.Append("\n  ]\n}\n");
            return builder.ToString();
        }

        static void AppendCase(StringBuilder builder, string caseId, bool diagnosticOnly, string purpose, Vector3 gravity, bool fixRoot, bool disableColliders, bool zeroOnly)
        {
            var previousGravity = Physics.gravity;
            Physics.gravity = gravity;
            try
            {
                builder.Append("    {\n");
                builder.Append("      \"case_id\": \"");
                builder.Append(caseId);
                builder.Append("\",\n      \"purpose\": \"");
                builder.Append(purpose);
                builder.Append("\",\n      \"diagnostic_only\": ");
                builder.Append(diagnosticOnly ? "true" : "false");
                builder.Append(",\n      \"marker\": \"");
                builder.Append(diagnosticOnly ? "DIAGNOSTIC ONLY" : "CURRENT CONDITION");
                builder.Append("\",\n      \"gravity_enabled\": ");
                builder.Append(gravity.sqrMagnitude > 0f ? "true" : "false");
                builder.Append(",\n      \"gravity_xyz\": [");
                builder.Append(Num(Physics.gravity.x));
                builder.Append(", ");
                builder.Append(Num(Physics.gravity.y));
                builder.Append(", ");
                builder.Append(Num(Physics.gravity.z));
                builder.Append("],\n      \"root_fixed_by_runner\": ");
                builder.Append(fixRoot ? "true" : "false");
                builder.Append(",\n      \"colliders_disabled\": ");
                builder.Append(disableColliders ? "true" : "false");
                builder.Append(",\n      \"configurations\": [\n");
                var configurations = zeroOnly ? new[] { Configurations()[0] } : Configurations();
                for (var index = 0; index < configurations.Length; index++)
                {
                    if (index > 0)
                    {
                        builder.Append(",\n");
                    }

                    AppendConfiguration(builder, configurations[index], fixRoot, disableColliders);
                }

                builder.Append("\n      ]\n    }");
            }
            finally
            {
                Physics.gravity = previousGravity;
            }
        }

        static void AppendConfiguration(StringBuilder builder, JointConfiguration configuration, bool fixRoot, bool disableColliders)
        {
            var instance = PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Ur10eGolden/ur10e.prefab")) as GameObject;
            try
            {
                Physics.SyncTransforms();
                var adapter = Ur10eJointAdapter.Bind(instance);
                var disabled = 0;
                if (disableColliders)
                {
                    disabled = adapter.DisableColliders();
                }

                var importedImmovable = adapter.RootImmovable();
                if (fixRoot)
                {
                    adapter.SetRootImmovable(true);
                }
                else
                {
                    adapter.SetRootImmovable(false);
                }

                var rootStart = adapter.RootWorldPosition();
                adapter.SetJointTargetsRadians(configuration.Joints);
                var step = Time.fixedDeltaTime;
                var steps = 0;
                var stableFrames = 0;
                var previous = adapter.GetJointPositionsRadians();
                var elapsed = 0.0;
                var settled = false;
                while (elapsed < Ur10eProjectTestParameters.TimeoutSeconds)
                {
                    Physics.Simulate(step);
                    steps++;
                    elapsed += step;
                    var actual = adapter.GetJointPositionsRadians();
                    var velocity = adapter.MaxAbsJointVelocityRadiansPerSecond();
                    var delta = 0.0;
                    for (var index = 0; index < actual.Length; index++)
                    {
                        delta = Math.Max(delta, Math.Abs(actual[index] - previous[index]));
                    }

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

                var samples = adapter.SampleJoints(configuration.Joints);
                var rootEnd = adapter.RootWorldPosition();
                var maxAbs = 0.0;
                builder.Append("        {\n          \"test_id\": \"");
                builder.Append(configuration.Id);
                builder.Append("\",\n          \"root_name\": \"");
                builder.Append(adapter.RootName());
                builder.Append("\",\n          \"root_imported_immovable\": ");
                builder.Append(importedImmovable ? "true" : "false");
                builder.Append(",\n          \"root_immovable_during_test\": ");
                builder.Append(adapter.RootImmovable() ? "true" : "false");
                builder.Append(",\n          \"root_displacement_m\": ");
                builder.Append(Num(Vector3.Distance(rootStart, rootEnd)));
                builder.Append(",\n          \"colliders_disabled_count\": ");
                builder.Append(disabled.ToString(CultureInfo.InvariantCulture));
                builder.Append(",\n          \"settling_steps\": ");
                builder.Append(steps.ToString(CultureInfo.InvariantCulture));
                builder.Append(",\n          \"settling_time\": ");
                builder.Append(Num(steps * step));
                builder.Append(",\n          \"timed_out\": ");
                builder.Append(settled ? "false" : "true");
                builder.Append(",\n          \"joints\": [\n");
                for (var index = 0; index < samples.Length; index++)
                {
                    if (index > 0)
                    {
                        builder.Append(",\n");
                    }

                    var sample = samples[index];
                    var error = sample.ActualRadians - sample.TargetRadians;
                    maxAbs = Math.Max(maxAbs, Math.Abs(error));
                    var ratio = sample.ForceLimit == 0.0 ? 0.0 : Math.Abs(sample.DriveForceNewtonMeters) / sample.ForceLimit;
                    builder.Append("            {\"name\": \"");
                    builder.Append(sample.Name);
                    builder.Append("\", \"target_rad\": ");
                    builder.Append(Num(sample.TargetRadians));
                    builder.Append(", \"actual_rad\": ");
                    builder.Append(Num(sample.ActualRadians));
                    builder.Append(", \"error_rad\": ");
                    builder.Append(Num(error));
                    builder.Append(", \"error_deg\": ");
                    builder.Append(Num(error * Ur10eJointAdapter.RadiansToDegrees));
                    builder.Append(", \"velocity_rad_s\": ");
                    builder.Append(Num(sample.VelocityRadiansPerSecond));
                    builder.Append(", \"stiffness\": ");
                    builder.Append(Num(sample.Stiffness));
                    builder.Append(", \"damping\": ");
                    builder.Append(Num(sample.Damping));
                    builder.Append(", \"force_limit\": ");
                    builder.Append(Num(sample.ForceLimit));
                    builder.Append(", \"drive_force_nm\": ");
                    builder.Append(sample.DriveForceReadable ? Num(sample.DriveForceNewtonMeters) : "null");
                    builder.Append(", \"drive_force_readable\": ");
                    builder.Append(sample.DriveForceReadable ? "true" : "false");
                    builder.Append(", \"drive_force_note\": \"");
                    builder.Append(sample.DriveForceNote);
                    builder.Append("\", \"joint_force_nm\": ");
                    builder.Append(sample.JointForceReadable ? Num(sample.JointForceNewtonMeters) : "null");
                    builder.Append(", \"joint_force_readable\": ");
                    builder.Append(sample.JointForceReadable ? "true" : "false");
                    builder.Append(", \"abs_drive_force_over_limit\": ");
                    builder.Append(sample.DriveForceReadable ? Num(ratio) : "null");
                    builder.Append(", \"mass_kg\": ");
                    builder.Append(Num(sample.Mass));
                    builder.Append(", \"inertia_xyz\": [");
                    builder.Append(Num(sample.InertiaX));
                    builder.Append(", ");
                    builder.Append(Num(sample.InertiaY));
                    builder.Append(", ");
                    builder.Append(Num(sample.InertiaZ));
                    builder.Append("]}");
                }

                builder.Append("\n          ],\n          \"max_abs_error_rad\": ");
                builder.Append(Num(maxAbs));
                builder.Append("\n        }");
            }
            finally
            {
                if (instance != null)
                {
                    UnityEngine.Object.DestroyImmediate(instance);
                }
            }
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

        static string Num(double value)
        {
            return value.ToString("G17", CultureInfo.InvariantCulture);
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
