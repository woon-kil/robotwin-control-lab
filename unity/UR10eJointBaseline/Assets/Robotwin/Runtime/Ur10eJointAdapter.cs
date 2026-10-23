using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

namespace Robotwin.Ur10e
{
    public sealed class Ur10eMappedJoint
    {
        public string CanonicalName;
        public string UnityPath;
        public string JointType;
        public int DofCount;
        public bool Immovable;
        public bool IsRoot;
        public double ImportedStiffness;
        public double ImportedDamping;
        public double ForceLimit;
        public double LowerLimitDegrees;
        public double UpperLimitDegrees;
        public Quaternion AnchorRotation;
        public ArticulationBody Body;
    }

    public readonly struct Ur10eTool0Pose
    {
        public readonly string Frame;
        public readonly Vector3 PositionMeters;
        public readonly Quaternion Rotation;

        public Ur10eTool0Pose(string frame, Vector3 positionMeters, Quaternion rotation)
        {
            Frame = frame;
            PositionMeters = positionMeters;
            Rotation = rotation;
        }
    }

    public sealed class Ur10eJointAdapter
    {
        public const double RadiansToDegrees = 180.0 / Math.PI;

        public static readonly string[] CanonicalOrder =
        {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        };

        readonly GameObject _robot;
        readonly Ur10eMappedJoint[] _joints;
        readonly Transform _tool0;

        Ur10eJointAdapter(GameObject robot, Ur10eMappedJoint[] joints, Transform tool0)
        {
            _robot = robot;
            _joints = joints;
            _tool0 = tool0;
        }

        public Ur10eMappedJoint[] Joints => _joints;

        public static Ur10eJointAdapter Bind(GameObject robot)
        {
            if (robot == null)
            {
                throw new Ur10eAdapterException("INVALID_MODEL", "robot instance is null");
            }

            var components = robot.GetComponentsInChildren<UrdfJoint>(true);
            var mapped = new Ur10eMappedJoint[CanonicalOrder.Length];
            for (var index = 0; index < CanonicalOrder.Length; index++)
            {
                var name = CanonicalOrder[index];
                UrdfJoint match = null;
                var count = 0;
                for (var componentIndex = 0; componentIndex < components.Length; componentIndex++)
                {
                    if (components[componentIndex].jointName != name)
                    {
                        continue;
                    }

                    count++;
                    match = components[componentIndex];
                }

                if (count != 1 || match == null)
                {
                    throw new Ur10eAdapterException("MAPPING", name + " mapping count is " + count.ToString(CultureInfo.InvariantCulture));
                }

                var body = match.GetComponent<ArticulationBody>();
                if (body == null)
                {
                    throw new Ur10eAdapterException("MAPPING", name + " has no ArticulationBody");
                }

                if (body.jointType != ArticulationJointType.RevoluteJoint)
                {
                    throw new Ur10eAdapterException("UNSUPPORTED_JOINT", name + " joint type is " + body.jointType);
                }

                if (body.dofCount != 1)
                {
                    throw new Ur10eAdapterException("UNSUPPORTED_JOINT", name + " dofCount is " + body.dofCount.ToString(CultureInfo.InvariantCulture));
                }

                var drive = body.xDrive;
                mapped[index] = new Ur10eMappedJoint
                {
                    CanonicalName = name,
                    UnityPath = PathOf(match.transform, robot.transform),
                    JointType = body.jointType.ToString(),
                    DofCount = body.dofCount,
                    Immovable = body.immovable,
                    IsRoot = body.isRoot,
                    ImportedStiffness = drive.stiffness,
                    ImportedDamping = drive.damping,
                    ForceLimit = drive.forceLimit,
                    LowerLimitDegrees = drive.lowerLimit,
                    UpperLimitDegrees = drive.upperLimit,
                    AnchorRotation = body.anchorRotation,
                    Body = body,
                };
            }

            Transform tool0 = null;
            var transforms = robot.GetComponentsInChildren<Transform>(true);
            for (var index = 0; index < transforms.Length; index++)
            {
                if (transforms[index].name != "tool0")
                {
                    continue;
                }

                if (tool0 != null)
                {
                    throw new Ur10eAdapterException("MAPPING", "tool0 is not unique");
                }

                tool0 = transforms[index];
            }

            if (tool0 == null)
            {
                throw new Ur10eAdapterException("MAPPING", "tool0 is missing");
            }

            return new Ur10eJointAdapter(robot, mapped, tool0);
        }

        public void SetJointTargetsRadians(double[] jointsRadians)
        {
            if (jointsRadians == null || jointsRadians.Length != CanonicalOrder.Length)
            {
                var length = jointsRadians == null ? "null" : jointsRadians.Length.ToString(CultureInfo.InvariantCulture);
                throw new Ur10eAdapterException("JOINT_COUNT", "expected 6 joints, got " + length);
            }

            for (var index = 0; index < jointsRadians.Length; index++)
            {
                if (double.IsNaN(jointsRadians[index]) || double.IsInfinity(jointsRadians[index]))
                {
                    throw new Ur10eAdapterException("NON_FINITE", CanonicalOrder[index] + " target is not finite");
                }
            }

            for (var index = 0; index < jointsRadians.Length; index++)
            {
                var body = _joints[index].Body;
                var drive = body.xDrive;
                drive.stiffness = Ur10eProjectTestParameters.DriveStiffness;
                drive.damping = Ur10eProjectTestParameters.DriveDamping;
                drive.target = (float)(jointsRadians[index] * RadiansToDegrees);
                drive.targetVelocity = 0f;
                body.xDrive = drive;
                body.WakeUp();
            }
        }

        public double[] GetJointPositionsRadians()
        {
            var positions = new double[CanonicalOrder.Length];
            for (var index = 0; index < _joints.Length; index++)
            {
                var body = _joints[index].Body;
                if (body.dofCount != 1)
                {
                    throw new Ur10eAdapterException("UNSUPPORTED_JOINT", _joints[index].CanonicalName + " dof changed");
                }

                positions[index] = body.jointPosition[0];
            }

            return positions;
        }

        public double MaxAbsJointVelocityRadiansPerSecond()
        {
            var max = 0.0;
            for (var index = 0; index < _joints.Length; index++)
            {
                var velocity = Math.Abs(_joints[index].Body.jointVelocity[0]);
                if (velocity > max)
                {
                    max = velocity;
                }
            }

            return max;
        }

        public Ur10eTool0Pose ReadTool0Pose()
        {
            return new Ur10eTool0Pose("unity_world", _tool0.position, _tool0.rotation);
        }

        public int DisableColliders()
        {
            var colliders = _robot.GetComponentsInChildren<Collider>(true);
            for (var index = 0; index < colliders.Length; index++)
            {
                colliders[index].enabled = false;
            }

            return colliders.Length;
        }

        public bool EnsureRootImmovable()
        {
            ArticulationBody root = null;
            var bodies = _robot.GetComponentsInChildren<ArticulationBody>(true);
            for (var index = 0; index < bodies.Length; index++)
            {
                if (!bodies[index].isRoot)
                {
                    continue;
                }

                if (root != null)
                {
                    throw new Ur10eAdapterException("INVALID_MODEL", "more than one articulation root");
                }

                root = bodies[index];
            }

            if (root == null)
            {
                throw new Ur10eAdapterException("INVALID_MODEL", "articulation root is missing");
            }

            var wasImmovable = root.immovable;
            root.immovable = true;
            return wasImmovable;
        }

        public string DescribeMapping()
        {
            var builder = new StringBuilder();
            for (var index = 0; index < _joints.Length; index++)
            {
                var joint = _joints[index];
                builder.Append(joint.CanonicalName);
                builder.Append(' ');
                builder.Append(joint.UnityPath);
                builder.Append('\n');
            }

            return builder.ToString();
        }

        static string PathOf(Transform transform, Transform root)
        {
            var names = new List<string>();
            var cursor = transform;
            while (cursor != null)
            {
                names.Add(cursor.name);
                if (cursor == root)
                {
                    break;
                }

                cursor = cursor.parent;
            }

            var builder = new StringBuilder();
            for (var index = names.Count - 1; index >= 0; index--)
            {
                if (builder.Length > 0)
                {
                    builder.Append('/');
                }

                builder.Append(names[index]);
            }

            return builder.ToString();
        }
    }
}
