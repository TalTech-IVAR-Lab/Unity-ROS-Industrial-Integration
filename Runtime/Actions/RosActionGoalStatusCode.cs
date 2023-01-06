namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Actions
{
    using System.Linq;
    using Common;

    public enum RosActionGoalStatusCodeEnum
    {
        /// <summary>
        /// The goal has yet to be processed by the action server.
        /// </summary>
        PENDING = 0,

        /// <summary>
        /// The goal is currently being processed by the action server.
        /// </summary>
        ACTIVE = 1,

        /// <summary>
        /// The goal received a cancel request after it started executing and has since completed its execution (Terminal State).
        /// </summary>
        PREEMPTED = 2,

        /// <summary>
        /// The goal was achieved successfully by the action server (Terminal State).
        /// </summary>
        SUCCEEDED = 3,

        /// <summary>
        /// The goal was aborted during execution by the action server due to some failure (Terminal State).
        /// </summary>
        ABORTED = 4,

        /// <summary>
        /// The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State).
        /// </summary>
        REJECTED = 5,

        /// <summary>
        /// The goal received a cancel request after it started executing and has not yet completed execution.
        /// </summary>
        PREEMPTING = 6,

        /// <summary>
        /// The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled.
        /// </summary>
        RECALLING = 7,

        /// <summary>
        /// The goal received a cancel request before it started executing and was successfully cancelled (Terminal State).
        /// </summary>
        RECALLED = 8,

        /// <summary>
        /// An action client can determine that a goal is LOST. This should not be sent over the wire by an action server.
        /// </summary>
        LOST = 9,
    }

    public class RosActionGoalStatusCode : StatusCodeBase<RosActionGoalStatusCodeEnum>
    {
        public RosActionGoalStatusCode(int value) : base(value) { }

        /// <summary>
        /// Status codes which correspond to the terminal states of the action.
        /// </summary>
        private static RosActionGoalStatusCodeEnum[] TerminalStateCodes =
        {
            RosActionGoalStatusCodeEnum.REJECTED,
            RosActionGoalStatusCodeEnum.RECALLED,
            RosActionGoalStatusCodeEnum.PREEMPTED,
            RosActionGoalStatusCodeEnum.ABORTED,
            RosActionGoalStatusCodeEnum.SUCCEEDED,
        };

        /// <summary>
        /// Checks if the given action status code is Terminal State.
        /// </summary>
        /// <remarks>
        /// http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html
        /// </remarks>
        public bool IsTerminal => TerminalStateCodes.Contains(value);

        /// <summary>
        /// Checks if the given action status code corresponds to success.
        /// </summary>
        public bool IsSuccessful => value == RosActionGoalStatusCodeEnum.SUCCEEDED;
    }
}