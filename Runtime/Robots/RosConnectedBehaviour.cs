namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using Unity.Robotics.ROSTCPConnector;
    using UnityEngine;

    /// <summary>
    /// Base class for behaviours which require a reference to RosConnection.
    /// </summary>
    public class RosConnectedBehaviour : MonoBehaviour
    {
        public ROSConnection rosConnection;
    }
}