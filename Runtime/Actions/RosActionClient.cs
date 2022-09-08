namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Actions
{
    using System;
    using System.Collections.Generic;
    using Cysharp.Threading.Tasks;
    using Cysharp.Threading.Tasks.Linq;
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
        where TActionGoal : ActionGoal<TGoal>, new()
        where TGoal : Message
        where TActionFeedback : ActionFeedback<TFeedback>
        where TFeedback : Message
        where TActionResult : ActionResult<TResult>
        where TResult : Message
    {
        #region Lifecycle

        public RosActionClient(ROSConnection rosConnection, string actionRootTopic)
        {
            RosConnection = rosConnection;
            ActionTopics = new RosActionTopics(actionRootTopic);

            // Register related topics
            rosConnection.RegisterPublisher<TActionGoal>(ActionTopics.goal);
            rosConnection.RegisterPublisher<TActionFeedback>(ActionTopics.feedback);
            rosConnection.RegisterPublisher<TActionResult>(ActionTopics.result);
            rosConnection.RegisterPublisher<GoalIDMsg>(ActionTopics.cancel);
            rosConnection.RegisterPublisher<GoalStatusArrayMsg>(ActionTopics.status);

            // Register listeners
            rosConnection.Subscribe<GoalStatusArrayMsg>(ActionTopics.status, ProcessStatusArray);
            rosConnection.Subscribe<TActionFeedback>(ActionTopics.feedback, ProcessFeedback);
            rosConnection.Subscribe<TActionResult>(ActionTopics.result, ProcessResult);
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

        #endregion

        #region Internal Variables

        private class ActionState
        {
            public List<GoalStatusMsg> statusHistory = new();
            public List<TActionFeedback> feedbackHistory = new();
            public TActionResult result;
        }

        /// <summary>
        /// Dictionary to hold the results of the actions that are currently being executed / finished execution.
        /// </summary>
        private Dictionary<string, ActionState> actionStates = new();

        #endregion

        #region Internal Methods

        private void ProcessStatusArray(GoalStatusArrayMsg statusArray)
        {
            foreach (var status in statusArray.status_list)
            {
                string goalID = status.goal_id.id;
                if (actionStates.ContainsKey(goalID)) { actionStates[goalID].statusHistory.Add(status); }
            }
        }

        private void ProcessFeedback(TActionFeedback feedback)
        {
            string goalID = feedback.status.goal_id.id;
            if (actionStates.ContainsKey(goalID)) { actionStates[goalID].feedbackHistory.Add(feedback); }
        }

        private void ProcessResult(TActionResult result)
        {
            string goalID = result.status.goal_id.id;
            if (actionStates.ContainsKey(goalID)) { actionStates[goalID].result = result; }
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
        public string InitiateAction(TGoal goal)
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

            // Add goal ID to dict to indicate that we are expecting a result for it
            actionStates.Add(goalID, new ActionState());

            // Publish action goal
            RosConnection.Publish(ActionTopics.goal, goalMsg);

            return goalID;
        }
        
        /// <summary>
        /// Awaits until the action with the given goal ID reaches terminal state.
        /// </summary>
        /// <param name="goalID">ID of the goal to await.</param>
        /// <returns>The result of the action.</returns>
        /// <exception cref="InvalidOperationException"></exception>
        public async UniTask<TActionResult> WaitUntilActionCompletes(string goalID)
        {
            if (!actionStates.ContainsKey(goalID))
            {
                throw new InvalidOperationException($"Goal ID '{goalID}' is not among the currently running actions, so it cannot be awaited.\n" +
                                                    $"Make sure that you use a goal ID for the existing goal (e.g. a result of InitiateAction() call).");
            }

            // Once the result is not null, the action is complete
            await UniTask.WaitUntil(() => IsActionComplete(goalID));

            // The action is complete and does not have to be tracked anymore, so we remove its goal ID from the dict
            var result = actionStates[goalID].result;
            actionStates.Remove(goalID);

            return result;
        }

        public IUniTaskAsyncEnumerable<GoalStatusMsg> MonitorActionStatus(string goalID)
        {
            return UniTaskAsyncEnumerable.Create<GoalStatusMsg>(async (writer, token) =>
            {
                int currentStatusIndex = 0;
                int lastYieldedStatusCode = int.MinValue;
                var statusHistory = actionStates[goalID].statusHistory;
                
                // Keep yielding new statuses as they come until the action is complete
                while (!IsActionComplete(goalID) && !token.IsCancellationRequested)
                {
                    int requiredHistoryCount = currentStatusIndex + 1;
                    await UniTask.WaitUntil(() => (statusHistory.Count >= requiredHistoryCount), cancellationToken: token);

                    var newStatus = statusHistory[currentStatusIndex];

                    if (newStatus.status != lastYieldedStatusCode)
                    {
                        await writer.YieldAsync(newStatus);
                        lastYieldedStatusCode = newStatus.status;
                    }

                    currentStatusIndex++;
                }
                
                // Publish the result status
                var resultStatus = actionStates[goalID].result.status;
                if (resultStatus.status != lastYieldedStatusCode) await writer.YieldAsync(resultStatus);
            });
        }

        /// <summary>
        /// Cancels action with the given goal ID.
        /// </summary>
        /// <param name="goalID">Goal ID to cancel.</param>
        /// <param name="cancelAllPreviousActions">If set, all goal IDs sent before this action will be cancelled, too.</param>
        /// <exception cref="NotImplementedException"></exception>
        public void CancelAction(string goalID, bool cancelAllPreviousActions = false)
        {
            if (IsActionComplete(goalID)) return;
            
            var goalIDMsg = new GoalIDMsg
            {
                id = goalID
            };

            if (cancelAllPreviousActions)
            {
                throw new NotImplementedException("Cancelling previous actions is not yet implemented.");
                // look here for possible implementation details: http://wiki.ros.org/actionlib/DetailedDescription
                // The question is how to handle the timestamp (either wait for the server to generate it, or use a local one?)
            }
            
            // Publish cancel request
            RosConnection.Publish(ActionTopics.cancel, goalIDMsg);
        }

        /// <summary>
        /// Checks whether action with the given goal ID is complete.
        /// </summary>
        /// <param name="goalID">Goal ID to check.</param>
        /// <returns>True if action is complete, false if it is not yet complete or not tracked by this client.</returns>
        private bool IsActionComplete(string goalID)
        {
            if (!actionStates.TryGetValue(goalID, out var state)) return false;
            
            return state.result != null;
        }

        #endregion
    }
}