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
            RobotJointContract.Metadata metadata = RobotJointContract.Capture(robotKinematics, nameof(robotKinematics));
            RobotJointContract.NamedVector vector = RobotJointContract.CaptureNamedVector(
                metadata,
                names,
                unityPositions,
                false,
                false,
                nameof(names),
                nameof(unityPositions),
                "Unity position");
            var rawPositions = new double[vector.Values.Length];
            for (int i = 0; i < vector.Values.Length; i++)
            {
                double position = vector.Values[i];
                bool isRevolute = metadata.Bodies[vector.RobotIndices[i]].jointType == ArticulationJointType.RevoluteJoint;
                if (isRevolute) { position = position / 180d * Math.PI; }
                // TODO: check if we should handle spherical joints, too

                if (!RobotJointContract.IsFinite(position))
                {
                    throw new ArgumentException($"Converted ROS position for '{names[i]}' must be finite.", nameof(unityPositions));
                }

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
            RobotJointContract.Metadata metadata = RobotJointContract.Capture(robotKinematics, nameof(robotKinematics));
            return RosToUnity(metadata, names, rosPositions);
        }

        internal static double[] RosToUnity(
            RobotJointContract.Metadata metadata,
            string[] names,
            double[] rosPositions)
        {
            double[] unityPositions = MapToRobotOrder(metadata, names, rosPositions, "position");

            for (int i = 0; i < metadata.Count; i++)
            {
                double position = unityPositions[i];

                bool isRevolute = metadata.Bodies[i].jointType == ArticulationJointType.RevoluteJoint;
                if (isRevolute) { position = position / Math.PI * 180d; }
                // TODO: check if we should handle spherical joints, too

                if (!RobotJointContract.IsFinite(position))
                {
                    throw new ArgumentException($"Converted Unity position for '{metadata.Names[i]}' must be finite.", nameof(rosPositions));
                }

                unityPositions[i] = position;
            }

            return unityPositions;
        }

        /// <summary>
        /// Maps values identified by joint name into the robot's joint order.
        /// </summary>
        public static double[] MapToRobotOrder(
            UrdfRobotKinematicsDataProvider robotKinematics,
            string[] names,
            double[] values,
            string valueName)
        {
            RobotJointContract.Metadata metadata = RobotJointContract.Capture(robotKinematics, nameof(robotKinematics));
            return MapToRobotOrder(metadata, names, values, valueName);
        }

        internal static double[] MapToRobotOrder(
            RobotJointContract.Metadata metadata,
            string[] names,
            double[] values,
            string valueName)
        {
            RobotJointContract.NamedVector vector = RobotJointContract.CaptureNamedVector(
                metadata,
                names,
                values,
                true,
                true,
                nameof(names),
                nameof(values),
                valueName);
            var orderedValues = new double[metadata.Count];
            for (int i = 0; i < vector.Values.Length; i++)
            {
                int robotIndex = vector.RobotIndices[i];
                if (robotIndex >= 0) { orderedValues[robotIndex] = vector.Values[i]; }
            }

            return orderedValues;
        }
    }
}
