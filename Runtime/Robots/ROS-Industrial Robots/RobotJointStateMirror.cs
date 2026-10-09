namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using UnityEngine;

    /// <summary>
    /// Mirrors validated ROS joint states to a Unity robot motion controller.
    /// </summary>
    public class RobotJointStateMirror : MonoBehaviour
    {
        /// <summary>
        /// Listener that owns the validated ROS joint-state telemetry.
        /// </summary>
        public RobotJointStatesListener jointStatesListener;

        /// <summary>
        /// Controller that applies mirrored targets to the virtual robot.
        /// </summary>
        public UrdfRobotMotionController motionController;

        private RobotJointStatesListener lastListener;

        private UrdfRobotMotionController lastMotionController;

        private ulong lastProcessedStateVersion;

        private void OnEnable()
        {
            lastListener = null;
            lastMotionController = null;
            lastProcessedStateVersion = 0;
        }

        private void Update()
        {
            if (jointStatesListener == null || motionController == null)
            {
                Fail("Both a joint-state listener and a motion controller are required.");
                return;
            }

            if (!ReferenceEquals(jointStatesListener, lastListener))
            {
                lastListener = jointStatesListener;
                lastProcessedStateVersion = 0;
            }

            if (!ReferenceEquals(motionController, lastMotionController))
            {
                lastMotionController = motionController;
                lastProcessedStateVersion = 0;
            }

            if (!ReferenceEquals(motionController.robot, jointStatesListener.unityRobot))
            {
                Fail("Motion controller must reference the listener's exact Unity robot provider.");
                return;
            }

            if (!motionController.isActiveAndEnabled) { return; }

            if (!jointStatesListener.TryCaptureLatestJointState(
                    lastProcessedStateVersion,
                    out ulong version,
                    out RobotJointContract.Metadata metadata,
                    out string[] names,
                    out double[] positions))
            {
                return;
            }

            lastProcessedStateVersion = version;
            try
            {
                motionController.SetJointTargets(metadata, names, positions);
            }
            catch (Exception exception)
            {
                jointStatesListener.InvalidateAcceptedJointState(version);
                Debug.LogWarning($"Rejected mirrored joint state: {exception.Message}", this);
            }
        }

        private void Fail(string message)
        {
            Debug.LogError($"Robot joint-state mirror disabled: {message}", this);
            enabled = false;
        }
    }
}
