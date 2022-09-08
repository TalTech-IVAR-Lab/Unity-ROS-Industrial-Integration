namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Actions
{
    using System;

    /// <summary>
    /// Convenience wrapper for accessing topics related to the given ROS action.
    /// </summary>
    [Serializable]
    public struct RosActionTopics
    {
        /// <param name="actionRootTopic">Root topic of this action.</param>
        public RosActionTopics(string actionRootTopic)
        {
            rootTopic = actionRootTopic;
            goal = $"{rootTopic}/goal";
            cancel = $"{rootTopic}/cancel";
            feedback = $"{rootTopic}/feedback";
            status = $"{rootTopic}/status";
            result = $"{rootTopic}/result";
        }

        public readonly string rootTopic;
        public readonly string goal;
        public readonly string cancel;
        public readonly string feedback;
        public readonly string status;
        public readonly string result;
    }
}