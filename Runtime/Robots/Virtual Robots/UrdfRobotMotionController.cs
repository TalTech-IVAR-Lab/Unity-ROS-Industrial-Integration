namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using System.Collections.Generic;
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

        /// <summary>
        /// Motion tolerance in degrees.
        /// </summary>
        /// <remarks>
        /// Used to check if the robot has reached the target position.
        /// </remarks>
        private const float MOTION_TOLERANCE = 0.01f;

        #endregion

        #region Unity Callbacks

        public void OnEnable() { InitializeJoints(); }

        private void FixedUpdate()
        {
            switch (controlMode)
            {
                case ControlMode.Kinematic:
                    UpdateArticulationBodiesKinematic();
                    break;
                case ControlMode.Physical:
                    UpdateArticulationBodiesPhysical();
                    break;
            }
        }

        #endregion

        #region Public Methods

        public (string[], double[]) GetJointPositions()
        {
            var jointPositions = new List<double>(robot.JointsCount);

            for (int i = 0; i < robot.JointsCount; i++)
            {
                var body = robot.jointArticulationBodies[i];
                float position = GetJointPosition(body);
                jointPositions.Add(position);
            }

            return (robot.jointNames.ToArray(), jointPositions.ToArray());
        }

        public async UniTask<bool> StopMotion()
        {
            (string[] _, double[] jointPositions) = GetJointPositions();
            jointPositions.CopyTo(jointTargets, 0);

            // TODO: Maybe check velocities here?
            await UniTask.WaitForFixedUpdate();

            return true;
        }

        public async UniTask<bool> Move(string[] jointNames, double[] positions)
        {
            try
            {
                for (int i = 0; i < robot.JointsCount; i++)
                for (int j = 0; j < jointNames.Length; j++)
                {
                    if (robot.jointNames[i] == jointNames[j]) { jointTargets[i] = (float)positions[j]; }
                }

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
                Debug.LogError("Caught exception when trying to move the robot.", this);
                Debug.LogException(e);
                return false;
            }

            return true;
        }

        #endregion

        #region Internal Methods

        private void InitializeJoints() { jointTargets = new float[robot.JointsCount]; }

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

        private void UpdateArticulationBodiesKinematic()
        {
            // Disable robot's articulation bodies to prevent physical motion
            foreach (var body in robot.articulationBodies) { body.enabled = false; }

            // Move the robot joints' bodies to the target positions
            for (int i = 0; i < robot.JointsCount; i++)
            {
                var body = robot.jointArticulationBodies[i];
                var jointTarget = jointTargets[i];
                var parentBody = body.transform.parent.GetComponentInParent<ArticulationBody>(true);

                if (parentBody == null)
                {
                    Debug.LogError($"'{body}' joint's articulation body is the root of articulation. This is not allowed for robot joints, as they have to be movable.\n" +
                                   $"Please make sure that robot has a base articulation body which is not a joint. ", this);
                    this.enabled = false;
                    break;
                }

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

        private void UpdateArticulationBodiesPhysical()
        {
            for (int i = 0; i < robot.JointsCount; i++)
            {
                var jointBody = robot.jointArticulationBodies[i];
                jointBody.enabled = true;

                var drive = jointBody.xDrive;
                drive.forceLimit = forceLimit;
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.target = jointTargets[i];
                jointBody.xDrive = drive;

                jointBody.jointFriction = jointFriction;
                jointBody.angularDamping = angularDamping;
            }
        }

        #endregion
    }
}