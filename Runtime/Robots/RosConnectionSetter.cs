namespace EE.TalTech.IVAR.Robotics.ROS
{
    using System.Collections.Generic;
    using Unity.Robotics.ROSTCPConnector;
    using UnityEngine;

    /// <summary>
    /// Script to simplify connecting multiple RosConnectedBehaviours to a single ROSConnection.
    /// </summary>
    // TODO: This won't work with interfaces (IRosConnected) if Malimbe doesn't get fixed: https://github.com/ExtendRealityLtd/Malimbe/issues/72
    public class RosConnectionSetter : MonoBehaviour
    {
        public ROSConnection rosConnection;

        /// <summary>
        /// GameObjects under which to look for RosConnectedBehaviours.
        /// </summary>
        public List<GameObject> roots = new List<GameObject>();

        private void Reset()
        {
            roots.Clear();
            roots.Add(gameObject);
        }

        private void Awake() { UpdateRosConnectionReferences(); }

        [ContextMenu("Update References")]
        private void UpdateRosConnectionReferences()
        {
            foreach (var root in roots)
            foreach (var connectedObject in root.GetComponentsInChildren<RosConnectedBehaviour>(true)) { connectedObject.rosConnection = rosConnection; }
        }
    }
}