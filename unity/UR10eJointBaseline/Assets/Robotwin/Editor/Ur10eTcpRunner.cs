using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace Robotwin.Ur10e.Editor
{
    public static class Ur10eTcpRunner
    {
        const string PrefabPath = "Assets/Ur10eGolden/ur10e.prefab";

        public static void Run()
        {
            Run(Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "evidence", "P-004", "raw")));
        }

        public static void Run(string evidenceDirectory)
        {
            var previousMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            try
            {
                Measure(evidenceDirectory);
            }
            finally
            {
                Physics.simulationMode = previousMode;
            }
        }

        static void Measure(string evidenceDirectory)
        {
            Directory.CreateDirectory(evidenceDirectory);
            var configurations = Configurations();
            var rows = new Sample[configurations.Length];
            for (var index = 0; index < configurations.Length; index++)
            {
                rows[index] = MeasureConfiguration(configurations[index]);
            }

            var rerun = MeasureConfiguration(configurations[0]);
            var builder = new StringBuilder();
            builder.Append("{\n");
            AppendString(builder, "gate", "P-004", true);
            AppendString(builder, "source", "unity", true);
            AppendString(builder, "source_commit", "7ce98d8c3512b9d0239da93b75de68ea64f61bd6", true);
            AppendString(builder, "unity_version", Application.unityVersion, true);
            AppendString(builder, "native_frame", "unity_world", true);
            AppendString(builder, "quaternion_order", "xyzw", true);
            AppendString(builder, "handedness", "left", true);
            AppendString(builder, "up_axis", "Y", true);
            builder.Append("  \"normalization_applied\": false,\n");
            builder.Append("  \"configurations\": [\n");
            for (var index = 0; index < rows.Length; index++)
            {
                if (index > 0)
                {
                    builder.Append(",\n");
                }

                WriteSample(builder, rows[index]);
            }

            builder.Append("\n  ],\n  \"rerun\": ");
            WriteSample(builder, rerun);
            builder.Append("\n}\n");
            File.WriteAllText(Path.Combine(evidenceDirectory, "unity_results.json"), builder.ToString());
        }

        static Sample MeasureConfiguration(JointConfiguration configuration)
        {
            var instance = InstantiateRobot();
            try
            {
                Physics.SyncTransforms();
                var adapter = Ur10eJointAdapter.Bind(instance);
                adapter.DisableColliders();
                adapter.EnsureRootImmovable();
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
                    var delta = MaxAbs(actual, previous);
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

                return new Sample
                {
                    Id = configuration.Id,
                    Target = configuration.Joints,
                    Actual = actual,
                    TimedOut = !settled,
                    Base = adapter.ReadFramePose("base_link"),
                    Tool0 = adapter.ReadTool0Pose(),
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

        static void WriteSample(StringBuilder builder, Sample sample)
        {
            builder.Append("{\n");
            AppendString(builder, "test_id", sample.Id, true, 2);
            AppendArray(builder, "target_joint_rad", sample.Target, true);
            AppendArray(builder, "actual_joint_rad", sample.Actual, true);
            AppendBool(builder, "timed_out", sample.TimedOut, true);
            WritePose(builder, "base_pose", sample.Base, true);
            WritePose(builder, "tool0_pose", sample.Tool0, false);
            builder.Append("    }");
        }

        static void WritePose(StringBuilder builder, string name, Ur10eTool0Pose pose, bool comma)
        {
            builder.Append("      \"");
            builder.Append(name);
            builder.Append("\": {\"frame\": \"unity_world\", \"position_m\": {\"x\": ");
            builder.Append(Num(pose.PositionMeters.x));
            builder.Append(", \"y\": ");
            builder.Append(Num(pose.PositionMeters.y));
            builder.Append(", \"z\": ");
            builder.Append(Num(pose.PositionMeters.z));
            builder.Append("}, \"quaternion_xyzw\": {\"x\": ");
            builder.Append(Num(pose.Rotation.x));
            builder.Append(", \"y\": ");
            builder.Append(Num(pose.Rotation.y));
            builder.Append(", \"z\": ");
            builder.Append(Num(pose.Rotation.z));
            builder.Append(", \"w\": ");
            builder.Append(Num(pose.Rotation.w));
            builder.Append("}}");
            if (comma)
            {
                builder.Append(',');
            }

            builder.Append('\n');
        }

        static void AppendString(StringBuilder builder, string name, string value, bool comma, int indent = 1)
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

        static void AppendBool(StringBuilder builder, string name, bool value, bool comma)
        {
            builder.Append("      \"");
            builder.Append(name);
            builder.Append("\": ");
            builder.Append(value ? "true" : "false");
            if (comma)
            {
                builder.Append(',');
            }

            builder.Append('\n');
        }

        static void AppendArray(StringBuilder builder, string name, double[] values, bool comma)
        {
            builder.Append("      \"");
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

        static double MaxAbs(double[] left, double[] right)
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

        sealed class Sample
        {
            public string Id;
            public double[] Target;
            public double[] Actual;
            public bool TimedOut;
            public Ur10eTool0Pose Base;
            public Ur10eTool0Pose Tool0;
        }

        sealed class JointConfiguration
        {
            public JointConfiguration(string id, double[] joints)
            {
                Id = id;
                Joints = joints;
            }

            public string Id { get; }

            public double[] Joints { get; }
        }
    }
}
