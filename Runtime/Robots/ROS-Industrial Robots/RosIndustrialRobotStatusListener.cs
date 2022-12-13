namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using RosMessageTypes.Industrial;
    using UnityEngine;

    public class RosIndustrialRobotStatusListener : RosConnectedBehaviour
    {
        #region Variables

        /// <summary>
        /// ROS topic of the joint states.
        /// </summary>
        public string robotStatusTopic = "robot_status";

        public RobotStatusMsg robotStatus;

        #endregion

        #region Unity Callbacks

        private void Start()
        {
            rosConnection.RegisterPublisher<RobotStatusMsg>(robotStatusTopic);
            rosConnection.Subscribe<RobotStatusMsg>(robotStatusTopic, UpdateRobotStatus);
        }

        #endregion

        #region Private Methods

        private void UpdateRobotStatus(RobotStatusMsg msg) { robotStatus = msg; }

        #endregion
    }
}