namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Tests
{
    using System.Collections.Generic;
    using System.Reflection;
    using System.Text.RegularExpressions;
    using NUnit.Framework;
    using RosMessageTypes.Sensor;
    using Unity.Robotics.UrdfImporter;
    using UnityEngine;
    using UnityEngine.TestTools;

    public class RobotJointStateMirrorTests
    {
        private readonly List<GameObject> gameObjects = new List<GameObject>();

        [TearDown]
        public void TearDown()
        {
            foreach (GameObject gameObject in gameObjects) { Object.DestroyImmediate(gameObject); }
            gameObjects.Clear();
        }

        [Test]
        public void AcceptedState_IsForwardedToMotionControllerOnUpdate()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, new JointStateMsg
            {
                name = new[] { "joint_2", "joint_1" },
                position = new[] { 0.25d, System.Math.PI / 2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            InvokeMirrorUpdate(mirror);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
        }

        [Test]
        public void PublicTelemetryMutation_DoesNotChangeCapturedMirrorTarget()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            listener.jointNames[0] = "tampered_joint";
            listener.unityJointPositions[0] = 999d;

            InvokeMirrorUpdate(mirror);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
        }

        [Test]
        public void AcceptedState_IsAppliedOnlyOnce()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            InvokeMirrorUpdate(mirror);
            controller.jointTargets = new[] { 10f, 20f };

            InvokeMirrorUpdate(mirror);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
        }

        [Test]
        public void ReplacingController_ReplaysLatestAcceptedState()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            InvokeMirrorUpdate(mirror);
            var replacementObject = new GameObject("replacement controller");
            replacementObject.SetActive(false);
            gameObjects.Add(replacementObject);
            var replacement = replacementObject.AddComponent<UrdfRobotMotionController>();
            replacement.robot = listener.unityRobot;
            replacement.jointTargets = new float[2];
            replacementObject.SetActive(true);
            mirror.motionController = replacement;

            InvokeMirrorUpdate(mirror);

            Assert.That(replacement.jointTargets, Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
        }

        [Test]
        public void ControllerForDifferentRobot_DisablesMirrorWithoutChangingTargets()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            var otherObject = new GameObject("other robot");
            otherObject.SetActive(false);
            gameObjects.Add(otherObject);
            otherObject.AddComponent<UrdfRobot>();
            var otherRobot = otherObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            otherRobot.jointNames = listener.unityRobot.jointNames;
            otherRobot.jointArticulationBodies = listener.unityRobot.jointArticulationBodies;
            controller.robot = otherRobot;
            LogAssert.Expect(LogType.Error, new Regex("^Robot joint-state mirror disabled:"));

            InvokeMirrorUpdate(mirror);

            Assert.That(mirror.enabled, Is.False);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
            Assert.That(listener.unityJointPositions, Is.EqualTo(new[] { 90d, 0.25d }).Within(0.000001d));
        }

        [Test]
        public void FloatUnrepresentableTarget_IsRejectedAndNextValidStateRecovers()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, new JointStateMsg
            {
                name = new[] { "joint_1", "joint_2" },
                position = new[] { 1e38d, 0.25d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected mirrored joint state:"));

            InvokeMirrorUpdate(mirror);

            Assert.That(mirror.enabled, Is.True);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
            Assert.That(listener.unityJointPositions[0], Is.GreaterThan((double)float.MaxValue));

            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            InvokeMirrorUpdate(mirror);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
        }

        [Test]
        public void NoAcceptedState_DoesNotChangeControllerTargets()
        {
            CreateRig(out RobotJointStatesListener _, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            controller.jointTargets = new[] { 10f, 20f };

            InvokeMirrorUpdate(mirror);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
            Assert.That(mirror.enabled, Is.True);
        }

        [Test]
        public void MetadataReorderAfterReceipt_IsRejectedWithoutPublishingTargets()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            string firstName = listener.unityRobot.jointNames[0];
            listener.unityRobot.jointNames[0] = listener.unityRobot.jointNames[1];
            listener.unityRobot.jointNames[1] = firstName;
            ArticulationBody firstBody = listener.unityRobot.jointArticulationBodies[0];
            listener.unityRobot.jointArticulationBodies[0] = listener.unityRobot.jointArticulationBodies[1];
            listener.unityRobot.jointArticulationBodies[1] = firstBody;
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected mirrored joint state:"));

            InvokeMirrorUpdate(mirror);

            Assert.That(mirror.enabled, Is.True);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
        }

        [Test]
        public void SameNameBodySubstitutionAfterReceipt_IsRejectedWithoutPublishingTargets()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            var substituteObject = new GameObject("substitute joint");
            gameObjects.Add(substituteObject);
            ArticulationBody substitute = substituteObject.AddComponent<ArticulationBody>();
            substitute.jointType = ArticulationJointType.RevoluteJoint;
            listener.unityRobot.jointArticulationBodies[0] = substitute;
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected mirrored joint state:"));

            InvokeMirrorUpdate(mirror);

            Assert.That(mirror.enabled, Is.True);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
        }

        [Test]
        public void ProviderReassignmentAfterReceipt_IsRejectedWithoutPublishingTargets()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            var replacementObject = new GameObject("replacement provider");
            replacementObject.SetActive(false);
            gameObjects.Add(replacementObject);
            replacementObject.AddComponent<UrdfRobot>();
            var replacement = replacementObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            replacement.jointNames = new List<string>(listener.unityRobot.jointNames);
            replacement.jointArticulationBodies = new List<ArticulationBody>(
                listener.unityRobot.jointArticulationBodies);
            listener.unityRobot = replacement;
            controller.robot = replacement;
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected mirrored joint state:"));

            InvokeMirrorUpdate(mirror);

            Assert.That(mirror.enabled, Is.True);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
        }

        [Test]
        public void JointTypeMutationAfterReceipt_IsRejectedWithoutPublishingTargets()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            listener.unityRobot.jointArticulationBodies[0].jointType = ArticulationJointType.PrismaticJoint;
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected mirrored joint state:"));

            InvokeMirrorUpdate(mirror);

            Assert.That(mirror.enabled, Is.True);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
        }

        [Test]
        public void RejectedMetadataReorder_CannotReplayAfterRestoreOrControllerReplacement()
        {
            AssertRejectedSnapshotRequiresFreshState(
                (listener, _) => SwapJointMetadata(listener.unityRobot),
                (listener, _) => SwapJointMetadata(listener.unityRobot));
        }

        [Test]
        public void RejectedBodySubstitution_CannotReplayAfterRestoreOrControllerReplacement()
        {
            ArticulationBody originalBody = null;
            AssertRejectedSnapshotRequiresFreshState(
                (listener, _) =>
                {
                    originalBody = listener.unityRobot.jointArticulationBodies[0];
                    var substituteObject = new GameObject("substitute joint");
                    gameObjects.Add(substituteObject);
                    ArticulationBody substitute = substituteObject.AddComponent<ArticulationBody>();
                    substitute.jointType = originalBody.jointType;
                    listener.unityRobot.jointArticulationBodies[0] = substitute;
                },
                (listener, _) => listener.unityRobot.jointArticulationBodies[0] = originalBody);
        }

        [Test]
        public void RejectedProviderReassignment_CannotReplayAfterRestoreOrControllerReplacement()
        {
            UrdfRobotKinematicsDataProvider originalProvider = null;
            AssertRejectedSnapshotRequiresFreshState(
                (listener, controller) =>
                {
                    originalProvider = listener.unityRobot;
                    var replacementObject = new GameObject("replacement provider");
                    replacementObject.SetActive(false);
                    gameObjects.Add(replacementObject);
                    replacementObject.AddComponent<UrdfRobot>();
                    var replacement = replacementObject.AddComponent<UrdfRobotKinematicsDataProvider>();
                    replacement.jointNames = new List<string>(originalProvider.jointNames);
                    replacement.jointArticulationBodies = new List<ArticulationBody>(
                        originalProvider.jointArticulationBodies);
                    listener.unityRobot = replacement;
                    controller.robot = replacement;
                },
                (listener, controller) =>
                {
                    listener.unityRobot = originalProvider;
                    controller.robot = originalProvider;
                });
        }

        [Test]
        public void RejectedJointTypeMutation_CannotReplayAfterRestoreOrControllerReplacement()
        {
            ArticulationJointType originalJointType = default;
            AssertRejectedSnapshotRequiresFreshState(
                (listener, _) =>
                {
                    originalJointType = listener.unityRobot.jointArticulationBodies[0].jointType;
                    listener.unityRobot.jointArticulationBodies[0].jointType =
                        ArticulationJointType.PrismaticJoint;
                },
                (listener, _) =>
                    listener.unityRobot.jointArticulationBodies[0].jointType = originalJointType);
        }

        [Test]
        public void DisabledController_DoesNotConsumeSnapshotAndReplaysWhenEnabled()
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            controller.enabled = false;
            InvokeListenerUpdate(listener, CreateAcceptedMessage());

            InvokeMirrorUpdate(mirror);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
            Assert.That(mirror.enabled, Is.True);

            controller.enabled = true;
            InvokeMirrorUpdate(mirror);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
        }

        private void CreateRig(
            out RobotJointStatesListener listener,
            out UrdfRobotMotionController controller,
            out RobotJointStateMirror mirror)
        {
            var robotObject = new GameObject("robot");
            robotObject.SetActive(false);
            gameObjects.Add(robotObject);
            robotObject.AddComponent<UrdfRobot>();
            var robot = robotObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            robot.jointNames = new List<string> { "joint_1", "joint_2" };
            robot.jointArticulationBodies = new List<ArticulationBody>();

            string[] names = { "joint_1", "joint_2" };
            ArticulationJointType[] types =
                { ArticulationJointType.RevoluteJoint, ArticulationJointType.PrismaticJoint };
            for (int i = 0; i < names.Length; i++)
            {
                var jointObject = new GameObject(names[i]);
                gameObjects.Add(jointObject);
                ArticulationBody body = jointObject.AddComponent<ArticulationBody>();
                body.jointType = types[i];
                robot.jointArticulationBodies.Add(body);
            }

            listener = robotObject.AddComponent<RobotJointStatesListener>();
            listener.unityRobot = robot;
            controller = robotObject.AddComponent<UrdfRobotMotionController>();
            controller.robot = robot;
            controller.jointTargets = new float[2];
            mirror = robotObject.AddComponent<RobotJointStateMirror>();
            mirror.jointStatesListener = listener;
            mirror.motionController = controller;
            robotObject.SetActive(true);
        }

        private void AssertRejectedSnapshotRequiresFreshState(
            System.Action<RobotJointStatesListener, UrdfRobotMotionController> mutateMetadata,
            System.Action<RobotJointStatesListener, UrdfRobotMotionController> restoreMetadata)
        {
            CreateRig(out RobotJointStatesListener listener, out UrdfRobotMotionController controller,
                out RobotJointStateMirror mirror);
            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            mutateMetadata(listener, controller);
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected mirrored joint state:"));

            InvokeMirrorUpdate(mirror);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
            restoreMetadata(listener, controller);
            controller.jointTargets = new[] { 10f, 20f };
            InvokeMirrorOnEnable(mirror);
            InvokeMirrorUpdate(mirror);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));

            var replacementObject = new GameObject("replacement controller");
            replacementObject.SetActive(false);
            gameObjects.Add(replacementObject);
            var replacement = replacementObject.AddComponent<UrdfRobotMotionController>();
            replacement.robot = listener.unityRobot;
            replacement.jointTargets = new float[2];
            replacementObject.SetActive(true);
            mirror.motionController = replacement;
            InvokeMirrorUpdate(mirror);
            Assert.That(replacement.jointTargets, Is.EqualTo(new[] { 0f, 0f }));

            InvokeListenerUpdate(listener, CreateAcceptedMessage());
            InvokeMirrorUpdate(mirror);
            Assert.That(replacement.jointTargets,
                Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
        }

        private static void SwapJointMetadata(UrdfRobotKinematicsDataProvider robot)
        {
            string firstName = robot.jointNames[0];
            robot.jointNames[0] = robot.jointNames[1];
            robot.jointNames[1] = firstName;
            ArticulationBody firstBody = robot.jointArticulationBodies[0];
            robot.jointArticulationBodies[0] = robot.jointArticulationBodies[1];
            robot.jointArticulationBodies[1] = firstBody;
        }

        private static JointStateMsg CreateAcceptedMessage()
        {
            return new JointStateMsg
            {
                name = new[] { "joint_2", "joint_1" },
                position = new[] { 0.25d, System.Math.PI / 2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            };
        }

        private static void InvokeListenerUpdate(RobotJointStatesListener listener, JointStateMsg message)
        {
            MethodInfo method = typeof(RobotJointStatesListener).GetMethod(
                "UpdateJointStates", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(method, Is.Not.Null);
            method.Invoke(listener, new object[] { message });
        }

        private static void InvokeMirrorUpdate(RobotJointStateMirror mirror)
        {
            MethodInfo method = typeof(RobotJointStateMirror).GetMethod(
                "Update", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(method, Is.Not.Null);
            method.Invoke(mirror, null);
        }

        private static void InvokeMirrorOnEnable(RobotJointStateMirror mirror)
        {
            MethodInfo method = typeof(RobotJointStateMirror).GetMethod(
                "OnEnable", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(method, Is.Not.Null);
            method.Invoke(mirror, null);
        }
    }
}
