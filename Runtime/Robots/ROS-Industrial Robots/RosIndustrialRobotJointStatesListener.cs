namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using System.Linq;
    using EE.TalTech.IVAR.Robotics;
    using RosMessageTypes.Sensor;
    using UnityEngine;
    using Zinnia.Data.Attribute;

    /// <summary>
    /// Listens to the state of the joints on the physical robot.
    /// </summary>
    public class RosIndustrialRobotJointStatesListener : RosConnectedBehaviour
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
        /// Joint positions as received from ROS
        /// (angular joint positions in radians, linear joint positions in meters).
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
            rosConnection.RegisterPublisher<JointStateMsg>(jointStatesTopic);
            rosConnection.Subscribe<JointStateMsg>(jointStatesTopic, UpdateJointStates);
        }

        #endregion

        #region Private Methods

        private void UpdateJointStates(JointStateMsg msg)
        {
            // check if joint names in ROS match the ones in Unity
            string[] rosJointNames = msg.name;
            if (!rosJointNames.SequenceEqual(unityRobot.jointNames))
            {
                string rosJointNamesString = string.Join(", ", rosJointNames);
                string unityJointNamesString = string.Join(", ", unityRobot.jointNames);

                Debug.LogWarning("Joint names received from ROS do not match the names on the virtual robot.\n" +
                                 "This will cause problems with Digital Twin control. Make sure that joint names in ROS and in Unity match exactly!\n" +
                                 $"  ROS joint names: {rosJointNamesString}\n" +
                                 $"  Unity joint names: {unityJointNamesString}\n");
                return;
            }

            jointNames = rosJointNames;
            rosJointPositions = msg.position;

            unityJointPositions = new double[jointNames.Length];
            for (int i = 0; i < jointNames.Length; i++)
            {
                double position = rosJointPositions[i];

                // TODO: check if we should handle spherical joints, too
                bool isRevolute = unityRobot.jointArticulationBodies[i].jointType == ArticulationJointType.RevoluteJoint;
                if (isRevolute) { position = position / Math.PI * 180; }

                unityJointPositions[i] = position;
            }

            jointVelocities = msg.velocity;
            jointEfforts = msg.effort;
        }

        #endregion
    }
}