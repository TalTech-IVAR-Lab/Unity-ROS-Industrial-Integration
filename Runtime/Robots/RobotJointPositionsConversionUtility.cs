namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using UnityEngine;

    /// <summary>
    /// 
    /// </summary>
    public static class RobotJointPositionsConversionUtility
    {
        /// <summary>
        /// Converts the given robot's joints' positions from Unity to ROS format.
        /// </summary>
        /// <param name="robotKinematics"><see cref="UrdfRobotKinematicsDataProvider"/> of the robot.</param>
        /// <param name="names">Joint names.</param>
        /// <param name="unityPositions">Joint positions in Unity format (degrees and meters).</param>
        /// <returns>Array of joint positions in ROS format (radians and meters).</returns>
        public static double[] UnityToRos(UrdfRobotKinematicsDataProvider robotKinematics, string[] names, double[] unityPositions)
        {
            double[] rawPositions = new double[unityPositions.Length];

            for (int i = 0; i < names.Length; i++)
            for (int j = 0; j < robotKinematics.jointNames.Count; j++)
            {
                if (names[i] != robotKinematics.jointNames[i]) continue;
                double position = unityPositions[i];

                bool isRevolute = robotKinematics.jointArticulationBodies[i].jointType == ArticulationJointType.RevoluteJoint;
                if (isRevolute) { position = position / 180d * Math.PI; }
                // TODO: check if we should handle spherical joints, too

                rawPositions[i] = position;
            }

            return rawPositions;
        }

        /// <summary>
        /// Converts the given robot's joints' positions from ROS to Unity format.
        /// </summary>
        /// <param name="robotKinematics"><see cref="UrdfRobotKinematicsDataProvider"/> of the robot.</param>
        /// <param name="names">Joint names.</param>
        /// <param name="rosPositions">Joint positions in ROS format (radians and meters).</param>
        /// <returns>Array of joint positions in Unity format (degrees and meters).</returns>
        public static double[] RosToUnity(UrdfRobotKinematicsDataProvider robotKinematics, string[] names, double[] rosPositions)
        {
            double[] unityPositions = new double[rosPositions.Length];

            for (int i = 0; i < names.Length; i++)
            for (int j = 0; j < robotKinematics.jointNames.Count; j++)
            {
                if (names[i] != robotKinematics.jointNames[i]) continue;

                double position = rosPositions[i];

                bool isRevolute = robotKinematics.jointArticulationBodies[i].jointType == ArticulationJointType.RevoluteJoint;
                if (isRevolute) { position = position / Math.PI * 180d; }
                // TODO: check if we should handle spherical joints, too

                unityPositions[i] = position;
            }

            return unityPositions;
        }
    }
}