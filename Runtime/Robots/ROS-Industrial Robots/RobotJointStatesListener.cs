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
        /// Motion controller that applies accepted mirror targets to the virtual robot.
        /// </summary>
        public UrdfRobotMotionController motionController;

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
                if (motionController != null && !ReferenceEquals(motionController.robot, unityRobot))
                {
                    throw new ArgumentException("Motion controller must reference the listener's exact Unity robot provider.", nameof(motionController));
                }

                double[] newRosPositions = RobotJointPositionsConversionUtility.MapToRobotOrder(
                    metadata, msg.name, msg.position, "position");
                double[] newUnityPositions = RobotJointPositionsConversionUtility.RosToUnity(
                    metadata, msg.name, msg.position);
                double[] newVelocities = MapOptionalVector(metadata, msg.name, msg.velocity, "velocity");
                double[] newEfforts = MapOptionalVector(metadata, msg.name, msg.effort, "effort");

                if (motionController != null)
                {
                    motionController.SetJointTargets(metadata.Names, newUnityPositions);
                }

                jointNames = (string[])metadata.Names.Clone();
                rosJointPositions = newRosPositions;
                unityJointPositions = newUnityPositions;
                jointVelocities = newVelocities;
                jointEfforts = newEfforts;
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

        #endregion
    }
}
