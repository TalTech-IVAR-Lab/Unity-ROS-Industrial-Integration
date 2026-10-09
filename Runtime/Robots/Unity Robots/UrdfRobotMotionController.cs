namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using System.Threading;
    using Cysharp.Threading.Tasks;
    using UnityEngine;

    public class UrdfRobotMotionController : MonoBehaviour, IRosRobotMotionController
    {
        #region Data Types

        public enum ControlMode
        {
            Physical,
            Kinematic
        }

        #endregion

        #region Variables

        public UrdfRobotKinematicsDataProvider robot;

        public ControlMode controlMode = ControlMode.Kinematic;

        [Header("Physical Control Parameters")]
        public float stiffness = 1f;

        public float damping = 1f;
        public float forceLimit = 1000f;
        public float jointFriction = 10f;
        public float angularDamping = 10f;

        // TODO: the values below should be read from URDF joints, not set globally for the robot
        // public float speed = 5f; // Units: degree/s
        // public float torque = 100f; // Units: Nm or N
        // public float acceleration = 5f;// Units: m/s^2 / degree/s^2

        /// <summary>
        /// Goal positions of the robot joints.
        /// </summary>
        public float[] jointTargets = Array.Empty<float>();

        private RobotJointContract.Metadata targetMetadataBinding;

        /// <summary>
        /// Motion tolerance in degrees.
        /// </summary>
        /// <remarks>
        /// Used to check if the robot has reached the target position.
        /// </remarks>
        private const float MOTION_TOLERANCE = 0.01f;

        private sealed class KinematicUpdateSnapshot
        {
            internal KinematicUpdateSnapshot(
                ArticulationBody[] allBodies,
                ArticulationBody[] jointBodies,
                ArticulationBody[] parentBodies,
                float[] targets)
            {
                AllBodies = allBodies;
                JointBodies = jointBodies;
                ParentBodies = parentBodies;
                Targets = targets;
            }

            internal ArticulationBody[] AllBodies { get; }
            internal ArticulationBody[] JointBodies { get; }
            internal ArticulationBody[] ParentBodies { get; }
            internal float[] Targets { get; }
        }

        private sealed class PhysicalUpdateSnapshot
        {
            internal PhysicalUpdateSnapshot(ArticulationBody[] jointBodies, float[] targets)
            {
                JointBodies = jointBodies;
                Targets = targets;
            }

            internal ArticulationBody[] JointBodies { get; }
            internal float[] Targets { get; }
        }

        #endregion

        #region Unity Callbacks

        public void OnEnable() { InitializeJoints(); }

        private void FixedUpdate()
        {
            try
            {
                switch (controlMode)
                {
                    case ControlMode.Kinematic:
                        UpdateArticulationBodiesKinematic(CaptureKinematicUpdate());
                        break;
                    case ControlMode.Physical:
                        UpdateArticulationBodiesPhysical(CapturePhysicalUpdate());
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }
            catch (Exception exception)
            {
                targetMetadataBinding = null;
                Debug.LogError($"Rejected robot controller FixedUpdate: {exception.Message}", this);
                enabled = false;
            }
        }

        #endregion

        #region Public Methods

        public (string[], double[]) GetJointPositions()
        {
            RobotJointContract.Metadata metadata = RobotJointContract.Capture(robot, nameof(robot));
            var jointPositions = new double[metadata.Count];

            for (int i = 0; i < metadata.Count; i++)
            {
                jointPositions[i] = GetJointPosition(metadata.Bodies[i]);
            }

            return ((string[])metadata.Names.Clone(), jointPositions);
        }

        public async UniTask<bool> StopMotion()
        {
            RobotJointContract.Metadata metadata = CaptureCurrentMetadata();
            var stoppedTargets = new float[metadata.Count];
            for (int i = 0; i < metadata.Count; i++)
            {
                float position = GetJointPosition(metadata.Bodies[i]);
                if (!RobotJointContract.IsFinite(position))
                {
                    throw new ArgumentException(
                        $"Current joint position for '{metadata.Names[i]}' must be finite.",
                        nameof(robot));
                }

                stoppedTargets[i] = position;
            }

            RobotJointContract.Metadata currentMetadata;
            try
            {
                currentMetadata = RobotJointContract.Capture(robot, nameof(robot));
            }
            catch
            {
                targetMetadataBinding = null;
                throw;
            }

            if (!RobotJointContract.MetadataMatches(metadata, currentMetadata))
            {
                targetMetadataBinding = null;
                throw new InvalidOperationException(
                    "Robot joint metadata changed while stopping motion.");
            }

            PublishTargets(metadata, stoppedTargets);

            // TODO: Maybe check velocities here?
            await UniTask.WaitForFixedUpdate();

            return true;
        }

        public async UniTask<bool> Move(string[] jointNames, double[] positions, CancellationToken cancellationToken = default)
        {
            try
            {
                SetJointTargets(jointNames, positions);

                // TODO: await until we reach the target?
                // TODO: create cancellation token each time we start a new motion to cancel the previous one? (otherwise loops will pile up)
                // while (true)
                // {
                //     bool reachedTarget = true;
                //
                //     (string[] _, double[] currentPositions) = GetJointPositions();
                //     for (int i = 0; i < robot.JointsCount; i++)
                //     {
                //         float currentPosition = (float)currentPositions[i];
                //         float targetPosition = jointTargets[i];
                //         if (Mathf.Abs(targetPosition - currentPosition) >= MOTION_TOLERANCE)
                //         {
                //             reachedTarget = false;
                //             break;
                //         }
                //     }
                //
                //     if (reachedTarget) { break; }
                //
                //     await UniTask.WaitForFixedUpdate();
                // }
            }
            catch (Exception e)
            {
                Debug.LogError($"Caught exception when trying to move the robot: {e.Message}", this);
                return false;
            }

            return true;
        }

        /// <summary>
        /// Validates and synchronously updates all robot joint targets by exact joint name.
        /// </summary>
        public void SetJointTargets(string[] jointNames, double[] positions)
        {
            RobotJointContract.Metadata metadata = CaptureCurrentMetadata();
            SetJointTargetsAgainstMetadata(metadata, jointNames, positions);
        }

        internal void SetJointTargets(
            RobotJointContract.Metadata expectedMetadata,
            string[] jointNames,
            double[] positions)
        {
            RobotJointContract.Metadata metadata = CaptureCurrentMetadata();
            if (!RobotJointContract.MetadataMatches(expectedMetadata, metadata))
            {
                throw new InvalidOperationException(
                    "Robot joint metadata changed after the mirrored state was accepted.");
            }

            SetJointTargetsAgainstMetadata(metadata, jointNames, positions);
        }

        private void SetJointTargetsAgainstMetadata(
            RobotJointContract.Metadata metadata,
            string[] jointNames,
            double[] positions)
        {
            RobotJointContract.NamedVector vector = RobotJointContract.CaptureNamedVector(
                metadata,
                jointNames,
                positions,
                false,
                false,
                nameof(jointNames),
                nameof(positions),
                "target position");
            var admittedValues = new float[vector.Values.Length];
            for (int i = 0; i < admittedValues.Length; i++)
            {
                admittedValues[i] = RobotJointContract.ToFiniteFloat(
                    vector.Values[i],
                    nameof(positions),
                    jointNames[i]);
            }

            float[] newTargets;
            if (vector.IsComplete)
            {
                newTargets = new float[metadata.Count];
            }
            else
            {
                if (!RobotJointContract.MetadataMatches(targetMetadataBinding, metadata))
                {
                    targetMetadataBinding = null;
                    throw new ArgumentException(
                        "Partial joint target updates require an exact admitted robot metadata binding.",
                        nameof(jointTargets));
                }

                if (jointTargets == null || jointTargets.Length != metadata.Count)
                {
                    throw new ArgumentException(
                        "Partial joint target updates require a baseline with exactly one target per robot joint.",
                        nameof(jointTargets));
                }

                for (int i = 0; i < jointTargets.Length; i++)
                {
                    if (!RobotJointContract.IsFinite(jointTargets[i]))
                    {
                        throw new ArgumentException(
                            $"Partial joint target baseline contains a non-finite value for '{metadata.Names[i]}'.",
                            nameof(jointTargets));
                    }
                }

                newTargets = (float[])jointTargets.Clone();
            }

            for (int i = 0; i < admittedValues.Length; i++)
            {
                newTargets[vector.RobotIndices[i]] = admittedValues[i];
            }

            if (vector.IsComplete)
            {
                PublishTargets(metadata, newTargets);
            }
            else
            {
                jointTargets = newTargets;
            }
        }

        #endregion

        #region Internal Methods

        private void InitializeJoints()
        {
            targetMetadataBinding = null;
            RobotJointContract.Metadata metadata = RobotJointContract.Capture(robot, nameof(robot));
            PublishTargets(metadata, new float[metadata.Count]);
        }

        /// <summary>
        /// Calculates robot's joint position based on it's ArticulationBody.
        /// </summary>
        /// <param name="jointBody">ArticulationBody of the joint.</param>
        // TODO: this should support other joint types + physical control mode 
        private float GetJointPosition(ArticulationBody jointBody)
        {
            switch (controlMode)
            {
                case ControlMode.Kinematic:
                    var parentBody = jointBody.transform.parent.GetComponentInParent<ArticulationBody>();

                    switch (jointBody.jointType)
                    {
                        case ArticulationJointType.RevoluteJoint:
                            var bodyTransformRotationWorld = jointBody.transform.rotation;
                            var anchorBaseRotationWorld =
                                parentBody.transform.rotation *
                                jointBody.parentAnchorRotation;
                            var anchorCurrentRotationWorld =
                                bodyTransformRotationWorld *
                                jointBody.anchorRotation;
                            var anchorRotationDelta =
                                Quaternion.Inverse(anchorBaseRotationWorld) *
                                anchorCurrentRotationWorld;
                            return anchorRotationDelta.eulerAngles.x;
                        case ArticulationJointType.SphericalJoint:
                            throw new NotImplementedException($"Kinematic motion control for {ArticulationJointType.SphericalJoint} is not yet implemented.");
                        case ArticulationJointType.PrismaticJoint:
                            throw new NotImplementedException($"Kinematic motion control for {ArticulationJointType.PrismaticJoint} is not yet implemented.");
                        case ArticulationJointType.FixedJoint:
                        default:
                            break;
                    }

                    break;
                case ControlMode.Physical:
                    throw new NotImplementedException($"{nameof(GetJointPosition)} is not yet implemented for {nameof(ControlMode.Physical)} control mode.");
                default:
                    throw new ArgumentOutOfRangeException();
            }

            return 0f;
        }

        private KinematicUpdateSnapshot CaptureKinematicUpdate()
        {
            RobotJointContract.Metadata metadata = CaptureCurrentMetadata();
            float[] targets = CapturePublicTargets(metadata);
            if (robot.articulationBodies == null)
            {
                throw new ArgumentException("Robot articulation bodies collection cannot be null.", nameof(robot));
            }

            var allBodies = new ArticulationBody[robot.articulationBodies.Count];
            for (int i = 0; i < allBodies.Length; i++)
            {
                allBodies[i] = robot.articulationBodies[i];
            }

            ValidateUnityIdentitySet(allBodies, "Robot articulation bodies");
            ArticulationBody[] hierarchyBodies = robot.GetComponentsInChildren<ArticulationBody>();
            ValidateUnityIdentitySet(hierarchyBodies, "Current robot hierarchy articulation bodies");

            var jointBodies = (ArticulationBody[])metadata.Bodies.Clone();
            var parentBodies = new ArticulationBody[metadata.Count];
            for (int i = 0; i < metadata.Count; i++)
            {
                if (!ContainsUnityIdentity(allBodies, jointBodies[i]))
                {
                    throw new ArgumentException(
                        $"'{metadata.Names[i]}' joint's articulation body must be present in the robot articulation bodies collection.",
                        nameof(robot));
                }

                if (jointBodies[i].jointType != ArticulationJointType.RevoluteJoint)
                {
                    throw new NotImplementedException(
                        $"Kinematic motion control for {jointBodies[i].jointType} is not yet implemented.");
                }

                Transform parentTransform = jointBodies[i].transform.parent;
                ArticulationBody parentBody = null;
                while (parentTransform != null && parentBody == null)
                {
                    parentBody = parentTransform.GetComponent<ArticulationBody>();
                    parentTransform = parentTransform.parent;
                }

                if (parentBody == null)
                {
                    throw new ArgumentException(
                        $"'{metadata.Names[i]}' joint's articulation body must have a parent articulation body.",
                        nameof(robot));
                }

                if (!ContainsUnityIdentity(allBodies, parentBody))
                {
                    throw new ArgumentException(
                        $"'{metadata.Names[i]}' joint's parent articulation body must be present in the robot articulation bodies collection.",
                        nameof(robot));
                }

                parentBodies[i] = parentBody;
            }

            if (allBodies.Length != hierarchyBodies.Length)
            {
                throw new ArgumentException(
                    "Robot articulation bodies count must match the current active robot hierarchy.",
                    nameof(robot));
            }

            for (int i = 0; i < allBodies.Length; i++)
            {
                if (!ContainsUnityIdentity(hierarchyBodies, allBodies[i]))
                {
                    throw new ArgumentException(
                        "Robot articulation bodies cannot contain a body outside the current active robot hierarchy.",
                        nameof(robot));
                }
            }

            for (int i = 0; i < hierarchyBodies.Length; i++)
            {
                if (!ContainsUnityIdentity(allBodies, hierarchyBodies[i]))
                {
                    throw new ArgumentException(
                        "Robot articulation bodies must contain every body in the current active robot hierarchy.",
                        nameof(robot));
                }
            }

            return new KinematicUpdateSnapshot(allBodies, jointBodies, parentBodies, targets);
        }

        private static void ValidateUnityIdentitySet(ArticulationBody[] bodies, string collectionName)
        {
            for (int i = 0; i < bodies.Length; i++)
            {
                if (bodies[i] == null)
                {
                    throw new ArgumentException(
                        $"{collectionName} cannot contain null or destroyed entries.",
                        nameof(robot));
                }

                for (int previous = 0; previous < i; previous++)
                {
                    if (bodies[previous] == bodies[i])
                    {
                        throw new ArgumentException(
                            $"{collectionName} cannot contain duplicate identities.",
                            nameof(robot));
                    }
                }
            }
        }

        private static bool ContainsUnityIdentity(ArticulationBody[] bodies, ArticulationBody requiredBody)
        {
            for (int i = 0; i < bodies.Length; i++)
            {
                if (bodies[i] == requiredBody) { return true; }
            }

            return false;
        }

        private PhysicalUpdateSnapshot CapturePhysicalUpdate()
        {
            RobotJointContract.Metadata metadata = CaptureCurrentMetadata();
            return new PhysicalUpdateSnapshot(metadata.Bodies, CapturePublicTargets(metadata));
        }

        private float[] CapturePublicTargets(RobotJointContract.Metadata metadata)
        {
            if (!RobotJointContract.MetadataMatches(targetMetadataBinding, metadata))
            {
                targetMetadataBinding = null;
                throw new ArgumentException(
                    "Public joint targets do not have an exact admitted robot metadata binding.",
                    nameof(jointTargets));
            }

            if (jointTargets == null || jointTargets.Length != metadata.Count)
            {
                throw new ArgumentException(
                    "Public joint targets must contain exactly one value per robot joint.",
                    nameof(jointTargets));
            }

            var targets = new float[jointTargets.Length];
            for (int i = 0; i < targets.Length; i++)
            {
                float target = jointTargets[i];
                if (!RobotJointContract.IsFinite(target))
                {
                    throw new ArgumentException(
                        $"Public joint target for '{metadata.Names[i]}' must be finite.",
                        nameof(jointTargets));
                }

                targets[i] = target;
            }

            return targets;
        }

        private RobotJointContract.Metadata CaptureCurrentMetadata()
        {
            RobotJointContract.Metadata metadata;
            try
            {
                metadata = RobotJointContract.Capture(robot, nameof(robot));
            }
            catch
            {
                targetMetadataBinding = null;
                throw;
            }

            if (targetMetadataBinding != null && !RobotJointContract.MetadataMatches(targetMetadataBinding, metadata))
            {
                targetMetadataBinding = null;
            }

            return metadata;
        }


        private void PublishTargets(RobotJointContract.Metadata metadata, float[] targets)
        {
            targetMetadataBinding = metadata;
            jointTargets = targets;
        }

        private void UpdateArticulationBodiesKinematic(KinematicUpdateSnapshot snapshot)
        {
            // Disable robot's articulation bodies to prevent physical motion
            foreach (ArticulationBody body in snapshot.AllBodies) { body.enabled = false; }

            // Move the robot joints' bodies to the target positions
            for (int i = 0; i < snapshot.JointBodies.Length; i++)
            {
                ArticulationBody body = snapshot.JointBodies[i];
                float jointTarget = snapshot.Targets[i];
                ArticulationBody parentBody = snapshot.ParentBodies[i];

                switch (body.jointType)
                {
                    case ArticulationJointType.RevoluteJoint:
                        var bodyTransformRotationWorld = body.transform.rotation;
                        var anchorBaseRotationWorld =
                            parentBody.transform.rotation *
                            body.parentAnchorRotation;
                        var anchorToTransformRotation =
                            Quaternion.Inverse(bodyTransformRotationWorld * body.anchorRotation) *
                            bodyTransformRotationWorld;
                        var anchorTargetRotationWorld =
                            anchorBaseRotationWorld *
                            Quaternion.Euler(jointTarget, 0f, 0f);
                        var targetRotation = anchorTargetRotationWorld * anchorToTransformRotation;

                        body.transform.rotation = targetRotation;
                        break;
                    case ArticulationJointType.SphericalJoint:
                        throw new NotImplementedException($"Kinematic motion control for {ArticulationJointType.SphericalJoint} is not yet implemented.");
                    case ArticulationJointType.PrismaticJoint:
                        throw new NotImplementedException($"Kinematic motion control for {ArticulationJointType.PrismaticJoint} is not yet implemented.");
                    case ArticulationJointType.FixedJoint:
                    default:
                        break;
                }
            }
        }

        private void UpdateArticulationBodiesPhysical(PhysicalUpdateSnapshot snapshot)
        {
            for (int i = 0; i < snapshot.JointBodies.Length; i++)
            {
                ArticulationBody jointBody = snapshot.JointBodies[i];
                jointBody.enabled = true;

                var drive = jointBody.xDrive;
                drive.forceLimit = forceLimit;
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.target = snapshot.Targets[i];
                jointBody.xDrive = drive;

                jointBody.jointFriction = jointFriction;
                jointBody.angularDamping = angularDamping;
            }
        }

        #endregion
    }
}
