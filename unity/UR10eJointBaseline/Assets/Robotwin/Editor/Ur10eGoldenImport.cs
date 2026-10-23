using System;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Xml;
using Robotwin.Ur10e;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEngine;

namespace Robotwin.Ur10e.Editor
{
    public static class Ur10eGoldenImport
    {
        const string UrdfAssetPath = "Assets/Ur10eGolden/ur10e.urdf";
        const string PrefabAssetPath = "Assets/Ur10eGolden/ur10e.prefab";
        const string ReportAssetPath = "Assets/Ur10eGolden/import_report.json";

        public static void Run()
        {
            try
            {
                Import();
                Debug.Log("UR10e golden import finished");
                EditorApplication.Exit(0);
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                File.WriteAllText(Path.Combine(Application.dataPath, "Ur10eGolden/import_error.txt"), exception.ToString());
                EditorApplication.Exit(1);
            }
        }

        static void Import()
        {
            AssetDatabase.Refresh();
            var urdfPath = Path.Combine(Application.dataPath, "Ur10eGolden/ur10e.urdf");
            if (!File.Exists(urdfPath))
            {
                throw new FileNotFoundException("URDF missing", urdfPath);
            }

            var settings = new ImportSettings
            {
                chosenAxis = ImportSettings.axisType.zAxis,
                convexMethod = ImportSettings.convexDecomposer.unity,
                OverwriteExistingPrefabs = true,
            };
            var pipeline = UrdfRobotExtensions.Create(urdfPath, settings, false, false);
            GameObject robot = null;
            while (pipeline.MoveNext())
            {
                if (pipeline.Current != null)
                {
                    robot = pipeline.Current;
                }
            }

            if (robot == null)
            {
                throw new InvalidOperationException("URDF importer returned no robot");
            }

            var adapter = Ur10eJointAdapter.Bind(robot);
            var prefab = PrefabUtility.SaveAsPrefabAsset(robot, PrefabAssetPath);
            if (prefab == null)
            {
                throw new InvalidOperationException("prefab save failed");
            }

            UnityEngine.Object.DestroyImmediate(robot);
            File.WriteAllText(ReportAssetPath, BuildReport(urdfPath, adapter, settings));
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
        }

        static string BuildReport(string urdfPath, Ur10eJointAdapter adapter, ImportSettings settings)
        {
            var builder = new StringBuilder();
            builder.Append("{\n");
            AppendString(builder, "unity_version", Application.unityVersion, 1, true);
            AppendString(builder, "operating_system", SystemInfo.operatingSystem, 1, true);
            AppendString(builder, "urdf_asset", UrdfAssetPath, 1, true);
            AppendString(builder, "prefab_asset", PrefabAssetPath, 1, true);
            AppendString(builder, "source_repository", "UniversalRobots/Universal_Robots_ROS2_Description", 1, true);
            AppendString(builder, "source_commit", "7ce98d8c3512b9d0239da93b75de68ea64f61bd6", 1, true);
            AppendString(builder, "robot_type", "ur10e", 1, true);
            AppendString(builder, "source_manifest", "robot_description/SOURCE_MANIFEST.md", 1, true);
            AppendString(builder, "urdf_sha256", Sha256(urdfPath), 1, true);
            AppendString(builder, "chosen_axis", settings.chosenAxis.ToString(), 1, true);
            AppendString(builder, "convex_method", settings.convexMethod.ToString(), 1, true);
            AppendString(builder, "urdf_importer_requested", "https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2", 1, true);
            AppendString(builder, "urdf_importer_lock_version", LockField("version"), 1, true);
            AppendString(builder, "urdf_importer_hash", Require(LockField("hash"), "importer hash"), 1, true);
            AppendString(builder, "urdf_importer_package_version", Require(PackageVersion(), "importer package version"), 1, true);
            builder.Append("  \"actuated_joints\": [\n");
            var document = new XmlDocument();
            document.Load(urdfPath);
            var nodes = document.GetElementsByTagName("joint");
            for (var index = 0; index < Ur10eJointAdapter.CanonicalOrder.Length; index++)
            {
                var name = Ur10eJointAdapter.CanonicalOrder[index];
                XmlElement match = null;
                for (var nodeIndex = 0; nodeIndex < nodes.Count; nodeIndex++)
                {
                    var element = nodes[nodeIndex] as XmlElement;
                    if (element != null && element.GetAttribute("name") == name)
                    {
                        match = element;
                        break;
                    }
                }

                if (match == null)
                {
                    throw new InvalidOperationException("URDF joint missing: " + name);
                }

                var mapped = adapter.Joints[index];
                if (index > 0)
                {
                    builder.Append(",\n");
                }

                builder.Append("    {\n");
                AppendString(builder, "canonical_name", name, 3, true);
                AppendString(builder, "unity_path", mapped.UnityPath, 3, true);
                AppendString(builder, "urdf_type", match.GetAttribute("type"), 3, true);
                AppendString(builder, "parent", ChildAttribute(match, "parent", "link"), 3, true);
                AppendString(builder, "child", ChildAttribute(match, "child", "link"), 3, true);
                AppendString(builder, "source_axis_xyz", ChildAttribute(match, "axis", "xyz"), 3, true);
                AppendString(builder, "unity_joint_type", mapped.JointType, 3, true);
                AppendNumber(builder, "unity_dof_count", mapped.DofCount, 3, true);
                AppendNumber(builder, "imported_stiffness", mapped.ImportedStiffness, 3, true);
                AppendNumber(builder, "imported_damping", mapped.ImportedDamping, 3, true);
                AppendNumber(builder, "force_limit", mapped.ForceLimit, 3, true);
                AppendNumber(builder, "lower_limit_deg", mapped.LowerLimitDegrees, 3, true);
                AppendNumber(builder, "upper_limit_deg", mapped.UpperLimitDegrees, 3, true);
                AppendNumber(builder, "anchor_rotation_x", mapped.AnchorRotation.x, 3, true);
                AppendNumber(builder, "anchor_rotation_y", mapped.AnchorRotation.y, 3, true);
                AppendNumber(builder, "anchor_rotation_z", mapped.AnchorRotation.z, 3, true);
                AppendNumber(builder, "anchor_rotation_w", mapped.AnchorRotation.w, 3, false);
                builder.Append("\n    }");
            }

            builder.Append("\n  ]\n}\n");
            return builder.ToString();
        }

        static string ChildAttribute(XmlElement joint, string childName, string attribute)
        {
            var child = joint[childName];
            if (child == null)
            {
                throw new InvalidOperationException(joint.GetAttribute("name") + " missing " + childName);
            }

            return child.GetAttribute(attribute);
        }

        static string LockField(string field)
        {
            var path = Path.GetFullPath(Path.Combine(Application.dataPath, "../Packages/packages-lock.json"));
            if (!File.Exists(path))
            {
                return "";
            }

            var text = File.ReadAllText(path);
            var packageIndex = text.IndexOf("\"com.unity.robotics.urdf-importer\"", StringComparison.Ordinal);
            if (packageIndex < 0)
            {
                return "";
            }

            var token = "\"" + field + "\"";
            var fieldIndex = text.IndexOf(token, packageIndex, StringComparison.Ordinal);
            if (fieldIndex < 0)
            {
                return "";
            }

            var quote = text.IndexOf('"', text.IndexOf(':', fieldIndex) + 1);
            var end = text.IndexOf('"', quote + 1);
            return text.Substring(quote + 1, end - quote - 1);
        }

        static string PackageVersion()
        {
            var cache = Path.GetFullPath(Path.Combine(Application.dataPath, "../Library/PackageCache"));
            if (!Directory.Exists(cache))
            {
                return "";
            }

            var directories = Directory.GetDirectories(cache, "com.unity.robotics.urdf-importer@*");
            if (directories.Length != 1)
            {
                return "";
            }

            var packageJson = File.ReadAllText(Path.Combine(directories[0], "package.json"));
            return LockFieldFromText(packageJson, "version");
        }

        static string LockFieldFromText(string text, string field)
        {
            var token = "\"" + field + "\"";
            var fieldIndex = text.IndexOf(token, StringComparison.Ordinal);
            var quote = text.IndexOf('"', text.IndexOf(':', fieldIndex) + 1);
            var end = text.IndexOf('"', quote + 1);
            return text.Substring(quote + 1, end - quote - 1);
        }

        static string Require(string value, string label)
        {
            if (string.IsNullOrEmpty(value))
            {
                throw new InvalidOperationException(label + " is empty");
            }

            return value;
        }

        static string Sha256(string path)
        {
            using (var stream = File.OpenRead(path))
            using (var hash = SHA256.Create())
            {
                var bytes = hash.ComputeHash(stream);
                var builder = new StringBuilder(bytes.Length * 2);
                for (var index = 0; index < bytes.Length; index++)
                {
                    builder.Append(bytes[index].ToString("x2", CultureInfo.InvariantCulture));
                }

                return builder.ToString();
            }
        }

        static void AppendString(StringBuilder builder, string name, string value, int indent, bool comma)
        {
            builder.Append(' ', indent * 2);
            builder.Append('"');
            builder.Append(name);
            builder.Append("\": \"");
            builder.Append(value.Replace("\\", "\\\\").Replace("\"", "\\\""));
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
            builder.Append(value.ToString("G17", CultureInfo.InvariantCulture));
            if (comma)
            {
                builder.Append(',');
            }

            builder.Append('\n');
        }
    }
}
