namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using Unity.Robotics.ROSTCPConnector;
    using Unity.Robotics.ROSTCPConnector.MessageGeneration;

    public static class RosConnectionExtensions
    {
        public static void RegisterAction<TGoal, TCancel, TFeedback, TStatus, TResult>(this ROSConnection rosConnection, RosAction action)
            where TGoal : Message
            where TCancel : Message
            where TFeedback : Message
            where TStatus : Message
            where TResult : Message
        {
            rosConnection.RegisterPublisher<TGoal>(action.GoalTopic);
            rosConnection.RegisterPublisher<TCancel>(action.CancelTopic);
            rosConnection.RegisterPublisher<TFeedback>(action.FeedbackTopic);
            rosConnection.RegisterPublisher<TStatus>(action.StatusTopic);
            rosConnection.RegisterPublisher<TResult>(action.ResultTopic);
        }

        [Serializable]
        public class RosAction
        {
            public RosAction(string actionTopic) { this.actionTopic = actionTopic; }

            private string actionTopic;

            public string GoalTopic => $"{actionTopic}/goal";
            public string CancelTopic => $"{actionTopic}/cancel";
            public string FeedbackTopic => $"{actionTopic}/feedback";
            public string StatusTopic => $"{actionTopic}/status";
            public string ResultTopic => $"{actionTopic}/result";
        }
    }
}