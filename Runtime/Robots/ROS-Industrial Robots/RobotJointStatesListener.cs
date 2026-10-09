namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using RosMessageTypes.Sensor;
    using UnityEngine;
    using Zinnia.Data.Attribute;

    /// <summary>
    /// Listens to the state of the joints on the physical robot.
    /// </summary>
    public class RobotJointStatesListener : RosConnectedBehaviour
    {
        #region Variables

        /// <summary>
        /// UrdfRobotKinematicsDataProvider of the corresponding virtual robot in Unity.
        /// </summary>
        public UrdfRobotKinematicsDataProvider unityRobot;


        /// <summary>
        /// ROS topic of the joint states.
        /// </summary>
        public string jointStatesTopic = "joint_states";

        /// <summary>
        /// Names of the robot's joints.
        /// </summary>
        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public string[] jointNames = Array.Empty<string>();

        /// <summary>
        /// Joint positions as received from ROS (angular joint positions in radians, linear joint positions in meters).
        /// </summary>
        /// <remarks>
        /// WARNING: These values must not be used as a reference for live robot control, as they are likely to be delayed due to network lag.
        /// </remarks>
        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public double[] rosJointPositions = Array.Empty<double>();

        /// <summary>
        /// Joint positions adjusted for Unity
        /// (angular joint positions in degrees, linear joint positions in meters).
        /// </summary>
        /// <remarks>
        /// WARNING: These values must not be used as a reference for live robot control, as they are likely to be delayed due to network lag.
        /// </remarks>
        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public double[] unityJointPositions = Array.Empty<double>();

        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public double[] jointVelocities = Array.Empty<double>();

        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public double[] jointEfforts = Array.Empty<double>();

        private double[] acceptedUnityJointPositions = Array.Empty<double>();

        private RobotJointContract.Metadata acceptedMetadata;

        private bool acceptedStateIsValid;

        private ulong acceptedStateVersion;

        #endregion

        #region Unity Callbacks

        private void Start()
        {
            var weakListener = new WeakReference<RobotJointStatesListener>(this);
            rosConnection.Subscribe<JointStateMsg>(jointStatesTopic, message =>
            {
                if (!weakListener.TryGetTarget(out RobotJointStatesListener listener)
                    || listener == null
                    || !listener.isActiveAndEnabled)
                {
                    return;
                }

                listener.UpdateJointStates(message);
            });
        }

        #endregion

        #region Private Methods

        private void UpdateJointStates(JointStateMsg msg)
        {
            try
            {
                RobotJointContract.Metadata metadata = RobotJointContract.Capture(unityRobot, nameof(unityRobot));
                double[] newRosPositions = RobotJointPositionsConversionUtility.MapToRobotOrder(
                    metadata, msg.name, msg.position, "position");
                double[] newUnityPositions = RobotJointPositionsConversionUtility.RosToUnity(
                    metadata, msg.name, msg.position);
                double[] newVelocities = MapOptionalVector(metadata, msg.name, msg.velocity, "velocity");
                double[] newEfforts = MapOptionalVector(metadata, msg.name, msg.effort, "effort");

                string[] newJointNames = (string[])metadata.Names.Clone();
                jointNames = newJointNames;
                rosJointPositions = newRosPositions;
                unityJointPositions = newUnityPositions;
                jointVelocities = newVelocities;
                jointEfforts = newEfforts;
                acceptedMetadata = metadata;
                acceptedUnityJointPositions = (double[])newUnityPositions.Clone();
                acceptedStateIsValid = true;
                unchecked
                {
                    acceptedStateVersion++;
                    if (acceptedStateVersion == 0) { acceptedStateVersion++; }
                }
            }
            catch (Exception exception)
            {
                Debug.LogWarning($"Rejected JointState message on '{jointStatesTopic}': {exception.Message}", this);
            }
        }

        private static double[] MapOptionalVector(
            RobotJointContract.Metadata metadata,
            string[] names,
            double[] values,
            string valueName)
        {
            if (values != null && values.Length == 0) { return Array.Empty<double>(); }
            return RobotJointPositionsConversionUtility.MapToRobotOrder(metadata, names, values, valueName);
        }

        /// <summary>
        /// Captures an immutable copy of the most recent accepted joint-state target.
        /// </summary>
        internal bool TryCaptureLatestJointState(
            ulong afterVersion,
            out ulong version,
            out RobotJointContract.Metadata metadata,
            out string[] names,
            out double[] positions)
        {
            version = acceptedStateVersion;
            if (version == 0 || version == afterVersion || !acceptedStateIsValid)
            {
                metadata = null;
                names = Array.Empty<string>();
                positions = Array.Empty<double>();
                return false;
            }

            metadata = acceptedMetadata;
            names = (string[])acceptedMetadata.Names.Clone();
            positions = (double[])acceptedUnityJointPositions.Clone();
            return true;
        }

        /// <summary>
        /// Permanently rejects an accepted snapshot if it is still the current version.
        /// A newer validated JointState is required before mirroring can resume.
        /// </summary>
        internal void InvalidateAcceptedJointState(ulong version)
        {
            if (acceptedStateVersion != version) { return; }
            acceptedStateIsValid = false;
            acceptedMetadata = null;
            acceptedUnityJointPositions = Array.Empty<double>();
        }

        #endregion
    }
}
