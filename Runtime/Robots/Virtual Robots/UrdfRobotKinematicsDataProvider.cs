namespace EE.TalTech.IVAR.Robotics.ROS
{
    using System.Collections.Generic;
    using System.Linq;
    using Unity.Robotics.UrdfImporter;
    using UnityEngine;
    using Zinnia.Data.Attribute;

    /// <summary>
    /// Provides info about kinematics of the robot imported from URDF.
    /// </summary>
    public class UrdfRobotKinematicsDataProvider : MonoBehaviour
    {
        #region Public Variables
        
        /// <summary>
        /// <see cref="UrdfRobot"/> script representing this robot.
        /// </summary>
        public UrdfRobot robot;

        /// <summary>
        /// List of <see cref="UrdfJoint"/>s of the robot.
        /// </summary>
        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public List<UrdfJoint> joints;

        /// <summary>
        /// List of names of robot's joints as defined in its URDF.
        /// </summary>
        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public List<string> jointNames;

        /// <summary>
        /// List of <see cref="ArticulationBody"/> scripts representing movable joints of the robot.
        /// </summary>
        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public List<ArticulationBody> jointArticulationBodies;

        /// <summary>
        /// List of names of robot's joints as defined in its URDF.
        /// </summary>
        [Restricted(RestrictedAttribute.Restrictions.ReadOnlyAlways)]
        public List<UrdfLink> links;

        /// <summary>
        /// Number of movable joints in this robot.
        /// </summary>
        public int JointsCount => jointArticulationBodies.Count;

        /// <summary>
        /// Last joint of the robots arm (the one to which the tool gets connected).
        /// </summary>
        public ArticulationBody ToolJoint => jointArticulationBodies[JointsCount - 1];

        /// <summary>
        /// Number of links in this robot.
        /// </summary>
        public int LinksCount => links.Count;

        /// <summary>
        /// Root <see cref="Transform"/> of the robot (base fixed joint).
        /// </summary>
        public UrdfLink RootLink => links[0];
        
        #endregion
        
        #region Unity Callbacks

        private void Reset()
        {
            robot = GetComponent<UrdfRobot>();
            CollectJoints();
            CollectLinks();
        }

        private void Awake()
        {
            CollectJoints();
            CollectLinks();
        }

        #endregion

        #region Internal Methods
        
        /// <summary>
        /// Collects references to the joints of the robot.
        /// </summary>
        private void CollectJoints()
        {
            // Find all joints
            joints = robot.GetComponentsInChildren<UrdfJoint>().ToList();
            
            // Find all dynamic joints of the robot
            var allJoints = robot.GetComponentsInChildren<UrdfJoint>(true);
            var dynamicJoints = allJoints.Where(joint => !(joint is UrdfJointFixed)).ToList();
            jointArticulationBodies = dynamicJoints.Select(joint => joint.GetComponent<ArticulationBody>()).ToList();
            
            // Find joint names
            jointNames = dynamicJoints.Select(joint => joint.jointName).ToList();
            
            Debug.Log($"Collected {JointsCount} dynamic joints on the robot '{robot.name}'.");
        }

        /// <summary>
        /// Collects references to the links of the robot.
        /// </summary>
        private void CollectLinks()
        {
            links = robot.GetComponentsInChildren<UrdfLink>().ToList();
            
            Debug.Log($"Collected {LinksCount} links on the robot '{robot.name}'.");
        }

        #endregion
    }
}
