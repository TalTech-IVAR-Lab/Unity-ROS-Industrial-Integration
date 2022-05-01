namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System.Collections.Generic;
    using System.Linq;
    // using RosMessageTypes.UnityMoveitIntegration;
    using Unity.Robotics.UrdfImporter;
    using Unity.Robotics.ROSTCPConnector;
    using UnityEngine;

    /// <summary>
    /// Publishes robots' joints to the given ROS topic.
    /// </summary>
    public class RosRobotJointsPublisher : MonoBehaviour
    {
        // ROS Connector
        private ROSConnection ros;

        // Variables required for ROS communication
        public string topicName = "test_joints";

        public GameObject robotRoot;

        // Articulation Bodies
        private readonly List<ArticulationBody> jointArticulationBodies = new List<ArticulationBody>();

        private int JointsCount => jointArticulationBodies.Count;

        /// <summary>
        /// 
        /// </summary>
        void Start()
        {
            // Get ROS connection static instance
            ros = ROSConnection.instance;

            // Find all dynamic joints of the robot
            var allJoints = robotRoot.GetComponentsInChildren<UrdfJoint>();
            var dynamicJoints = allJoints.Where(joint => !(joint is UrdfJointFixed)).ToList();

            foreach (var joint in dynamicJoints) { jointArticulationBodies.Add(joint.GetComponent<ArticulationBody>()); }

            Debug.Log($"Found {JointsCount} dynamic joints on {robotRoot.name} robot.");
        }

        // public void Publish()
        // {
        //     var jointsMessage = new MoveitJointsMsg
        //     {
        //         // Record all joint values
        //         joints = new double[JointsCount]
        //     };
        //
        //     for (int i = 0; i < JointsCount; i++)
        //     {
        //         jointsMessage.joints[i] = jointArticulationBodies[i].xDrive.target;
        //     }
        //
        //     // // Pick Pose
        //     // jointsMessage.pick_pose = new RosMessageTypes.Geometry.Pose
        //     // {
        //     //     position = target.transform.position.To<FLU>(),
        //     //     orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
        //     // };
        //     //
        //     // // Place Pose
        //     // jointsMessage.place_pose = new RosMessageTypes.Geometry.Pose
        //     // {
        //     //     position = targetPlacement.transform.position.To<FLU>(),
        //     //     orientation = pickOrientation.To<FLU>()
        //     // };
        //
        //     // Finally send the message to server_endpoint.py running in ROS
        //     ros.Send(topicName, jointsMessage);
        // }
    }
}