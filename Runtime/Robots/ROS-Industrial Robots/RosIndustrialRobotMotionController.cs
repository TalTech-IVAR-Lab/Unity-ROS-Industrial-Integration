namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using System.Linq;
    using Actions;
    using Cysharp.Threading.Tasks;
    using RosMessageTypes.Actionlib;
    using RosMessageTypes.Control;
    using RosMessageTypes.Industrial;
    using RosMessageTypes.Std;
    using RosMessageTypes.Trajectory;
    using UnityEngine;
    using Common;
    using RosMessageTypes.BuiltinInterfaces;

    /// <summary>
    /// Interface for controlling the motion of a robot connected through ROS-Industrial.
    /// </summary>
    public class RosIndustrialRobotMotionController : RosConnectedBehaviour, IRosRobotMotionController
    {
        #region Variables

        [Header("ROS-Industrial Listeners")]
        public RosIndustrialRobotStatusListener robotStatusListener;

        public RosIndustrialRobotJointStatesListener jointStatesListener;

        /// <summary>
        /// ROS topic to enable the robot.
        /// </summary>
        [Header("ROS Topics")]
        public string robotEnableServiceTopic = "robot_enable";

        /// <summary>
        /// ROS topic to disable the robot.
        /// </summary>
        public string robotDisableServiceTopic = "robot_disable";

        /// <summary>
        /// ROS topic to stop current robot motion.
        /// </summary>
        public string robotStopMotionServiceTopic = "stop_motion";

        /// <summary>
        /// ROS topic to execute motion on the robot.
        /// </summary>
        public string robotPathCommandServiceTopic = "joint_path_command";

        /// <summary>
        /// TODO: docs (joint_trajectory_action from http://wiki.ros.org/industrial_robot_client/generic_implementation)
        /// </summary>
        public string robotJointTrajectoryActionTopic = "joint_trajectory_action";

        #endregion

        #region Unity Callbacks

        private void OnEnable()
        {
            rosConnection.RegisterRosService<TriggerRequest, TriggerResponse>(robotEnableServiceTopic);
            rosConnection.RegisterRosService<TriggerRequest, TriggerResponse>(robotDisableServiceTopic);
            rosConnection.RegisterRosService<StartMotionRequest, StopMotionResponse>(robotStopMotionServiceTopic);
            rosConnection.RegisterRosService<CmdJointTrajectoryRequest, CmdJointTrajectoryResponse>(robotPathCommandServiceTopic);
            rosConnection.RegisterRosAction<
                FollowJointTrajectoryActionGoal,
                FollowJointTrajectoryActionFeedback,
                FollowJointTrajectoryActionResult
            >(robotJointTrajectoryActionTopic);
        }

        #endregion

        #region Public Methods (State)

        public void EnableNoWait() { EnableRobot().Forget(); }

        public void DisableNoWait() { DisableRobot().Forget(); }

        public async UniTask<TriggerResponse> EnableRobot()
        {
            var response = await rosConnection.SendServiceMessage<TriggerResponse>(robotEnableServiceTopic, new TriggerRequest());

            if (!response.success) { Debug.LogError($"Error when trying to enable robot: {response.message}", this); }
            else { Debug.Log("Robot enabled successfully.", this); }

            return response;
        }

        public async UniTask<TriggerResponse> DisableRobot()
        {
            var response = await rosConnection.SendServiceMessage<TriggerResponse>(robotDisableServiceTopic, new TriggerRequest());

            if (!response.success) { Debug.LogError($"Error when trying to disable robot: {response.message}", this); }
            else { Debug.Log("Robot disabled successfully.", this); }

            return response;
        }

        #endregion

        #region Public Methods (Motion)

        public async UniTask<bool> StopMotion()
        {
            var response = await rosConnection.SendServiceMessage<StopMotionResponse>(robotStopMotionServiceTopic, new StopMotionRequest());

            return (response.code.val == ServiceReturnCodeMsg.SUCCESS);
        }

        // TODO: figure out what's better: service or action (joint_path_command Move() implementation below)
        public async UniTask<bool> Move(string[] names, double[] positions)
        {
            Debug.Log("Moving robot to position (Unity): " + string.Join(", ", positions));
            positions = jointStatesListener.ConvertToRawPositions(names, positions);
            Debug.Log("Moving robot to position (ROS): " + string.Join(", ", positions));

            var goal = new FollowJointTrajectoryActionGoal
            {
                goal = new FollowJointTrajectoryGoal
                {
                    trajectory = new JointTrajectoryMsg
                    {
                        joint_names = names,
                        points = new[]
                        {
                            new JointTrajectoryPointMsg()
                            {
                                positions = positions
                            }
                        }
                    }
                }
            };

            string moveGoalID = rosConnection.PublishActionGoal(robotJointTrajectoryActionTopic, goal);

            void ProcessStatusUpdate(GoalStatusArrayMsg statusArray)
            {
                foreach (var status in statusArray.status_list)
                {
                    if (status.goal_id.id != moveGoalID) continue;

                    var actionStatusCode = new RosActionGoalStatusCode(status.status);
                    Debug.Log($"Move action status update: {status.status} ({actionStatusCode} - {status.text})");

                    if (RosActionGoalStatusCode.IsTerminal(status.status)) { Debug.Log($"Move action has reached terminal state with status '{actionStatusCode}'."); }
                }
            }

            // rosConnection.Subscribe<GoalStatusArrayMsg>(action.StatusTopic, ProcessStatusUpdate);

            return false;
        }

        /// <summary>
        /// Robot's joints positions adjusted for Unity (angular joint positions in degrees, linear joint positions in meters).
        /// </summary>
        /// <returns>Arrays of joint names and positions.</returns>
        public (string[], double[]) GetJointPositions() { return ((string[])jointStatesListener.jointNames.Clone(), (double[])jointStatesListener.jointPositions.Clone()); }

        #endregion
    }
}