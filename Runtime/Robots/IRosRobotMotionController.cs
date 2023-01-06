namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System.Threading;
    using Cysharp.Threading.Tasks;

    // TODO: find a way to serialize interfaces in the Inspector (so that we could plug controller references in there)
    public interface IRosRobotMotionController
    {
        public (string[], double[]) GetJointPositions();

        public UniTask<bool> StopMotion();

        public UniTask<bool> Move(string[] jointNames, double[] positions, CancellationToken cancellationToken);

        // public UniTask<bool> MoveJoint(string name, double position);
    }
}