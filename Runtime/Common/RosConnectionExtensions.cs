namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using Actions;
    using RosMessageTypes.Actionlib;
    using RosMessageTypes.BuiltinInterfaces;
    using Unity.Robotics.ROSTCPConnector;
    using Unity.Robotics.ROSTCPConnector.MessageGeneration;

    /// <summary>
    /// Extension methods for <see cref="ROSConnection"/> class.
    /// </summary>
    public static class RosConnectionExtensions
    {
        /// <summary>
        /// Registers a new ROS action.
        /// </summary>
        /// <param name="rosConnection"><see cref="ROSConnection"/> to register the action with.</param>
        /// <param name="topic">Root ROS topic of the action.</param>
        /// <typeparam name="TGoal">Action's goal message type.</typeparam>
        /// <typeparam name="TFeedback">Action's feedback message type.</typeparam>
        /// <typeparam name="TResult">Action's result message type.</typeparam>
        public static void RegisterRosAction<TGoal, TFeedback, TResult>(this ROSConnection rosConnection, string topic)
            where TGoal : Message
            where TFeedback : Message
            where TResult : Message
        {
            var action = new RosAction(topic);

            rosConnection.RegisterPublisher<TGoal>(action.GoalTopic);
            rosConnection.RegisterPublisher<TFeedback>(action.FeedbackTopic);
            rosConnection.RegisterPublisher<TResult>(action.ResultTopic);
            rosConnection.RegisterPublisher<GoalIDMsg>(action.CancelTopic);
            rosConnection.RegisterPublisher<GoalStatusArrayMsg>(action.StatusTopic);
        }

        /// <summary>
        /// Publishes a goal message to the given acton topic.
        /// Goal ID is generated automatically.
        /// </summary>
        /// <returns>Unique ID of the published goal.</returns>
        public static string PublishActionGoal<TMessage>(this ROSConnection rosConnection, 
            string actionRootTopic, 
            ActionGoal<TMessage> message) where TMessage : Message
        {
            // Generate and set goal identification info
            string goalID = Guid.NewGuid().ToString();
            message.goal_id = new GoalIDMsg
            {
                id = goalID,
                // stamp - let the action server handle the timestamp 
            };

            // Publish action
            var action = new RosAction(actionRootTopic);
            rosConnection.Publish(action.GoalTopic, message);

            return goalID;
        }
    }
}