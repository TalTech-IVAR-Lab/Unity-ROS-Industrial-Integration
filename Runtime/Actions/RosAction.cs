namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Actions
{
    using System;

    /// <summary>
    /// Convenience wrapper for accessing topics related to the given ROS Action.
    /// </summary>
    [Serializable]
    public class RosAction
    {
        public RosAction(string actionRootTopic) { this.actionRootTopic = actionRootTopic; }

        private string actionRootTopic;

        public string GoalTopic => $"{actionRootTopic}/goal";
        public string CancelTopic => $"{actionRootTopic}/cancel";
        public string FeedbackTopic => $"{actionRootTopic}/feedback";
        public string StatusTopic => $"{actionRootTopic}/status";
        public string ResultTopic => $"{actionRootTopic}/result";
    }
}