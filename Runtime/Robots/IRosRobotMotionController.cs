namespace EE.TalTech.IVAR.Robotics.ROS
{
    using Cysharp.Threading.Tasks;

    // TODO: find a way to serialize interfaces in the Inspector (so that we could plug controller references in there)
    public interface IRosRobotMotionController
    {
        public (string[], double[]) GetJointPositions();

        public UniTask<bool> StopMotion();

        public UniTask<bool> Move(string[] jointNames, double[] positions);

        // public UniTask<bool> MoveJoint(string name, double position);
    }
}