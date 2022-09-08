namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Actions
{
    using System;
    using System.Collections.Generic;
    using Cysharp.Threading.Tasks;
    using RosMessageTypes.Actionlib;
    using Unity.Robotics.ROSTCPConnector;
    using Unity.Robotics.ROSTCPConnector.MessageGeneration;
    using UnityEngine;

    /// <summary>
    /// Provides methods for convenient communication with ROS action servers.
    /// </summary>
    /// <remarks>
    /// http://wiki.ros.org/actionlib
    /// </remarks>
    public class RosActionClient<TActionGoal, TActionFeedback, TActionResult, TGoal, TFeedback, TResult> : IDisposable
        where TActionGoal : ActionGoal<TGoal>, new() where TGoal : Message
        where TActionFeedback : ActionFeedback<TFeedback> where TFeedback : Message
        where TActionResult : ActionResult<TResult> where TResult : Message
    {
        #region Lifecycle

        public RosActionClient(ROSConnection rosConnection, string actionRootTopic)
        {
            RosConnection = rosConnection;
            ActionRootTopic = actionRootTopic;
            ActionTopics = new RosActionTopics(actionRootTopic);

            // Register related topics
            rosConnection.RegisterPublisher<TActionGoal>(ActionTopics.goal);
            rosConnection.RegisterPublisher<TActionFeedback>(ActionTopics.feedback);
            rosConnection.RegisterPublisher<TActionResult>(ActionTopics.result);
            rosConnection.RegisterPublisher<GoalIDMsg>(ActionTopics.cancel);
            rosConnection.RegisterPublisher<GoalStatusArrayMsg>(ActionTopics.status);
            
            // Register listeners
            rosConnection.Subscribe<TActionFeedback>(ActionTopics.feedback, (_) => {});
            rosConnection.Subscribe<TActionResult>(ActionTopics.result, ProcessResult);
            rosConnection.Subscribe<GoalStatusArrayMsg>(ActionTopics.status, ProcessStatus);
        }

        public void Dispose()
        {
            RosConnection.Unsubscribe(ActionTopics.feedback);
            RosConnection.Unsubscribe(ActionTopics.result);
            RosConnection.Unsubscribe(ActionTopics.status);
        }

        #endregion

        #region Public Variables

        public RosActionTopics ActionTopics { get; }
        public ROSConnection RosConnection { get; }
        public string ActionRootTopic { get; }

        #endregion

        #region Internal Variables

        private Dictionary<string, Action<TActionResult>> resultCallbacks = new();
        private Dictionary<string, Action<GoalStatusMsg>> statusCallbacks = new();
        // private Dictionary<string, Action<TResult>> feedbackCallbacks = new(); // implement later if the need for feedback arises

        #endregion

        #region Internal Methods

        private void ProcessStatus(GoalStatusArrayMsg statusArray)
        {
            foreach (var status in statusArray.status_list)
            {
                string goalID = status.goal_id.id;
                
                if (statusCallbacks.ContainsKey(goalID))
                {
                    statusCallbacks[goalID]?.Invoke(status);
                }
            }
        }

        private void ProcessResult(TActionResult result)
        {
            string goalID = result.status.goal_id.id;
            if (resultCallbacks.ContainsKey(goalID))
            {
                // Invoke the last status update callback
                statusCallbacks[goalID]?.Invoke(result.status);
                statusCallbacks.Remove(goalID);
                
                // Invoke the result callback
                resultCallbacks[goalID]?.Invoke(result);
                resultCallbacks.Remove(goalID);
            }
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Publishes a goal to the given acton topic.
        /// </summary>
        /// <remarks>
        /// Goal ID is generated automatically.
        /// </remarks>
        /// <returns>Unique ID of the published goal.</returns>
        public string InitiateAction(
            TGoal goal,
            Action<TActionResult> onCompletion = null, 
            Action<GoalStatusMsg> onStatusUpdate = null
            // Action<TFeedback> onFeedback = null // implement later if the need for feedback arises
        )
        {
            // Generate and set goal identification info
            string goalID = Guid.NewGuid().ToString();
            var goalMsg = new TActionGoal
            {
                goal_id = new GoalIDMsg
                {
                    id = goalID,
                    // stamp - let the action server handle the timestamp 
                },
                goal = goal
            };
            
            // Register callbacks
            resultCallbacks.Add(goalID, onCompletion);
            statusCallbacks.Add(goalID, onStatusUpdate);
            
            // Publish action goal
            RosConnection.Publish(ActionTopics.goal, goalMsg);

            return goalID;
        }
        
        /// <summary>
        /// Submits the given goal to the action server and returns the result once it is available.
        /// </summary>
        /// <param name="goal">Goal of the action to be executed.</param>
        /// <returns>Result of the action.</returns>
        public async UniTask<TActionResult> ExecuteAction(TGoal goal)
        {
            bool isCompleted = false;
            TActionResult actionResult = null;
            string goalID = null;

            void OnCompletion(TActionResult result)
            {
                isCompleted = true;
                actionResult = result;
            }

            void OnStatusUpdate(GoalStatusMsg status)
            {
                var code = new RosActionGoalStatusCode(status.status);
                Debug.Log($"Received status update for goal ID '{goalID}':\n" +
                          $"[{status.status}/{code}] {status.text}");
            }

            goalID = InitiateAction(goal, OnCompletion, OnStatusUpdate);

            await UniTask.WaitUntil(() => isCompleted);

            if (actionResult == null) throw new Exception($"Could not receive action result for goal ID '{goalID}'.");

            return actionResult;
        }

        #endregion
    }
}