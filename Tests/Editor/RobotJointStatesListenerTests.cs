namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Tests
{
    using System.Collections;
    using System.Collections.Generic;
    using System.IO;
    using System.Reflection;
    using System.Text.RegularExpressions;
    using NUnit.Framework;
    using RosMessageTypes.Sensor;
    using Unity.Robotics.ROSTCPConnector;
    using Unity.Robotics.UrdfImporter;
    using UnityEditor.PackageManager;
    using UnityEngine;
    using UnityEngine.TestTools;

    public class RobotJointStatesListenerTests
    {
        private readonly List<GameObject> gameObjects = new List<GameObject>();

        [TearDown]
        public void TearDown()
        {
            foreach (GameObject gameObject in gameObjects) { Object.DestroyImmediate(gameObject); }
            gameObjects.Clear();
        }

        [Test]
        public void CanonicalRobotJointStatesListenerTypeExists()
        {
            Assert.That(
                typeof(UrdfRobotMotionController).Assembly.GetType(
                    "EE.TalTech.IVAR.Robotics.ROSIndustrial.RobotJointStatesListener"),
                Is.Not.Null);
        }

        [Test]
        public void AcceptedReorderedState_UpdatesPublicStateAndMotionControllerTargetsByName()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.PrismaticJoint },
                out UrdfRobotMotionController controller);
            var message = new JointStateMsg
            {
                name = new[] { "joint_2", "extra_joint", "joint_1" },
                position = new[] { 0.25d, 9d, System.Math.PI / 2d },
                velocity = new[] { 2d, 99d, 1d },
                effort = new[] { 4d, 99d, 3d }
            };

            InvokeUpdate(listener, message);

            Assert.That(listener.jointNames, Is.EqualTo(new[] { "joint_1", "joint_2" }));
            Assert.That(listener.rosJointPositions, Is.EqualTo(new[] { System.Math.PI / 2d, 0.25d }));
            Assert.That(listener.unityJointPositions, Is.EqualTo(new[] { 90d, 0.25d }).Within(0.000001d));
            Assert.That(listener.jointVelocities, Is.EqualTo(new[] { 1d, 2d }));
            Assert.That(listener.jointEfforts, Is.EqualTo(new[] { 3d, 4d }));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
        }

        [Test]
        public void AcceptedStateWithoutMotionController_StillUpdatesPublicState()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.PrismaticJoint },
                out UrdfRobotMotionController controller);
            listener.motionController = null;
            var message = new JointStateMsg
            {
                name = new[] { "joint_2", "joint_1" },
                position = new[] { 0.25d, System.Math.PI / 2d },
                velocity = new[] { 2d, 1d },
                effort = new[] { 4d, 3d }
            };

            InvokeUpdate(listener, message);

            Assert.That(listener.jointNames, Is.EqualTo(new[] { "joint_1", "joint_2" }));
            Assert.That(listener.rosJointPositions, Is.EqualTo(new[] { System.Math.PI / 2d, 0.25d }));
            Assert.That(listener.unityJointPositions, Is.EqualTo(new[] { 90d, 0.25d }).Within(0.000001d));
            Assert.That(listener.jointVelocities, Is.EqualTo(new[] { 1d, 2d }));
            Assert.That(listener.jointEfforts, Is.EqualTo(new[] { 3d, 4d }));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f, 0f }));
        }

        [Test]
        public void EmptyVelocityAndEffort_AreAcceptedAndRemainEmpty()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.PrismaticJoint },
                out UrdfRobotMotionController controller);
            var message = new JointStateMsg
            {
                name = new[] { "joint_2", "joint_1" },
                position = new[] { 0.25d, System.Math.PI / 2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            };

            InvokeUpdate(listener, message);

            Assert.That(listener.jointVelocities, Is.Empty);
            Assert.That(listener.jointEfforts, Is.Empty);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 90f, 0.25f }).Within(0.000001f));
        }

        [Test]
        public void WithoutController_ConversionOverflow_IsRejectedAtomically()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController controller);
            listener.motionController = null;
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected JointState message"));

            InvokeUpdate(listener, new JointStateMsg
            {
                name = new[] { "joint_1" },
                position = new[] { double.MaxValue },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            Assert.That(listener.jointNames, Is.Empty);
            Assert.That(listener.rosJointPositions, Is.Empty);
            Assert.That(listener.unityJointPositions, Is.Empty);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f }));
        }

        [Test]
        public void WithController_FloatUnrepresentableConvertedTarget_IsRejectedAtomically()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController controller);
            float[] originalTargets = controller.jointTargets;
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected JointState message"));

            InvokeUpdate(listener, new JointStateMsg
            {
                name = new[] { "joint_1" },
                position = new[] { 1e38d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            Assert.That(listener.jointNames, Is.Empty);
            Assert.That(listener.rosJointPositions, Is.Empty);
            Assert.That(listener.unityJointPositions, Is.Empty);
            Assert.That(controller.jointTargets, Is.SameAs(originalTargets));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f }));
        }

        [Test]
        public void WithoutController_LargeFiniteConvertedDouble_IsAccepted()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController controller);
            listener.motionController = null;

            InvokeUpdate(listener, new JointStateMsg
            {
                name = new[] { "joint_1" },
                position = new[] { 1e38d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            Assert.That(listener.jointNames, Is.EqualTo(new[] { "joint_1" }));
            Assert.That(listener.rosJointPositions, Is.EqualTo(new[] { 1e38d }));
            Assert.That(listener.unityJointPositions[0], Is.GreaterThan((double)float.MaxValue));
            Assert.That(double.IsInfinity(listener.unityJointPositions[0]), Is.False);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f }));
        }

        [Test]
        public void ControllerForDifferentRobot_IsRejectedAtomically()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController controller);
            var otherObject = new GameObject("other robot");
            otherObject.SetActive(false);
            gameObjects.Add(otherObject);
            otherObject.AddComponent<UrdfRobot>();
            var otherRobot = otherObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            otherRobot.jointNames = new List<string> { "joint_1" };
            otherRobot.jointArticulationBodies = new List<ArticulationBody>
            {
                listener.unityRobot.jointArticulationBodies[0]
            };
            controller.robot = otherRobot;
            float[] originalTargets = controller.jointTargets;
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected JointState message"));

            InvokeUpdate(listener, new JointStateMsg
            {
                name = new[] { "joint_1" },
                position = new[] { 0.5d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            Assert.That(listener.jointNames, Is.Empty);
            Assert.That(listener.rosJointPositions, Is.Empty);
            Assert.That(listener.unityJointPositions, Is.Empty);
            Assert.That(controller.jointTargets, Is.SameAs(originalTargets));
        }

        [Test]
        public void InvalidRobotMetadata_IsRejectedAtomicallyByListener()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController _);
            listener.motionController = null;
            UrdfRobotKinematicsDataProvider robot = listener.unityRobot;
            List<string> names = robot.jointNames;
            List<ArticulationBody> bodies = robot.jointArticulationBodies;
            var message = new JointStateMsg
            {
                name = new[] { "joint_1", "joint_2" },
                position = new[] { 0.1d, 0.2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            };

            robot.jointNames = null;
            AssertRejectedWithoutPublishing(listener, message);
            robot.jointNames = names;

            robot.jointArticulationBodies = null;
            AssertRejectedWithoutPublishing(listener, message);
            robot.jointArticulationBodies = bodies;

            foreach (string invalidName in new string[] { null, "" })
            {
                names[0] = invalidName;
                AssertRejectedWithoutPublishing(listener, message);
            }
            names[0] = "joint_1";

            names[1] = "joint_1";
            AssertRejectedWithoutPublishing(listener, message);
            names[1] = "joint_2";

            ArticulationBody firstBody = bodies[0];
            bodies[0] = null;
            AssertRejectedWithoutPublishing(listener, message);
            bodies[0] = firstBody;

            bodies.RemoveAt(1);
            AssertRejectedWithoutPublishing(listener, message);
            bodies.Add(firstBody);

            Object.DestroyImmediate(firstBody);
            AssertRejectedWithoutPublishing(listener, message);
        }

        [Test]
        public void DuplicateLiveArticulationBody_IsRejectedAtomicallyByListener()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController controller);
            ArticulationBody originalSecondBody = listener.unityRobot.jointArticulationBodies[1];
            listener.unityRobot.jointArticulationBodies[1] = listener.unityRobot.jointArticulationBodies[0];
            float[] originalTargets = controller.jointTargets;
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected JointState message"));

            InvokeUpdate(listener, new JointStateMsg
            {
                name = new[] { "joint_1", "joint_2" },
                position = new[] { 0.1d, 0.2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            Assert.That(listener.jointNames, Is.Empty);
            Assert.That(listener.rosJointPositions, Is.Empty);
            Assert.That(listener.unityJointPositions, Is.Empty);
            Assert.That(controller.jointTargets, Is.SameAs(originalTargets));
            Assert.That(originalSecondBody, Is.Not.Null);
        }

        [TestCase("missing")]
        [TestCase("duplicate")]
        [TestCase("position-length")]
        [TestCase("velocity-length")]
        [TestCase("effort-length")]
        [TestCase("null-names")]
        [TestCase("null-position")]
        [TestCase("null-velocity")]
        [TestCase("null-effort")]
        [TestCase("empty-name")]
        [TestCase("position-nan")]
        [TestCase("position-positive-infinity")]
        [TestCase("position-negative-infinity")]
        [TestCase("velocity-nan")]
        [TestCase("effort-positive-infinity")]
        public void MalformedState_IsRejectedAtomicallyWithOneWarning(string malformedField)
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.PrismaticJoint },
                out UrdfRobotMotionController controller);
            var validMessage = new JointStateMsg
            {
                name = new[] { "joint_1", "joint_2" },
                position = new[] { System.Math.PI / 2d, 0.25d },
                velocity = new[] { 1d, 2d },
                effort = new[] { 3d, 4d }
            };
            InvokeUpdate(listener, validMessage);

            string[] acceptedNames = (string[])listener.jointNames.Clone();
            double[] acceptedRosPositions = (double[])listener.rosJointPositions.Clone();
            double[] acceptedUnityPositions = (double[])listener.unityJointPositions.Clone();
            double[] acceptedVelocities = (double[])listener.jointVelocities.Clone();
            double[] acceptedEfforts = (double[])listener.jointEfforts.Clone();
            float[] acceptedTargets = (float[])controller.jointTargets.Clone();

            JointStateMsg malformedMessage = CreateMalformedMessage(malformedField);
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected JointState message"));
            InvokeUpdate(listener, malformedMessage);

            Assert.That(listener.jointNames, Is.EqualTo(acceptedNames));
            Assert.That(listener.rosJointPositions, Is.EqualTo(acceptedRosPositions));
            Assert.That(listener.unityJointPositions, Is.EqualTo(acceptedUnityPositions));
            Assert.That(listener.jointVelocities, Is.EqualTo(acceptedVelocities));
            Assert.That(listener.jointEfforts, Is.EqualTo(acceptedEfforts));
            Assert.That(controller.jointTargets, Is.EqualTo(acceptedTargets));
        }

        [Test]
        public void EnabledListener_ProcessesSubscribedCallbacks()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController controller);
            ROSConnection connection = SubscribeListener(listener);
            listener.gameObject.SetActive(true);

            DeliverSubscribedMessage(connection, listener.jointStatesTopic, new JointStateMsg
            {
                name = new[] { "joint_1" },
                position = new[] { System.Math.PI / 2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            Assert.That(listener.rosJointPositions, Is.EqualTo(new[] { System.Math.PI / 2d }));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 90f }).Within(0.000001f));
        }

        [Test]
        public void DisabledListener_IgnoresSubscribedCallbacks()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController controller);
            ROSConnection connection = SubscribeListener(listener);
            listener.enabled = false;

            DeliverSubscribedMessage(connection, listener.jointStatesTopic, new JointStateMsg
            {
                name = new[] { "joint_1" },
                position = new[] { System.Math.PI / 2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            Assert.That(listener.jointNames, Is.Empty);
            Assert.That(listener.rosJointPositions, Is.Empty);
            Assert.That(listener.unityJointPositions, Is.Empty);
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f }));
        }

        [Test]
        public void DestroyedListener_IgnoresSubscribedCallbacks()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController controller);
            ROSConnection connection = SubscribeListener(listener);
            listener.gameObject.SetActive(true);
            Object.DestroyImmediate(listener);

            DeliverSubscribedMessage(connection, "joint_states", new JointStateMsg
            {
                name = new[] { "joint_1" },
                position = new[] { System.Math.PI / 2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            });

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f }));
        }

        [Test]
        public void SubscriptionCallback_DoesNotStronglyRetainListener()
        {
            RobotJointStatesListener listener = CreateListener(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint },
                out UrdfRobotMotionController _);
            ROSConnection connection = SubscribeListener(listener);

            System.Delegate callback = GetSubscribedCallback(connection, listener.jointStatesTopic);
            Assert.That(DelegateGraphDirectlyReferences(callback, listener), Is.False);
        }

        [Test]
        public void LegacyListenerName_IsAnObsoleteCanonicalAlias()
        {
#pragma warning disable 618
            Assert.That(typeof(RosIndustrialRobotJointStatesListener).IsSubclassOf(typeof(RobotJointStatesListener)), Is.True);
            Assert.That(typeof(RosIndustrialRobotJointStatesListener).GetCustomAttribute<System.ObsoleteAttribute>(), Is.Not.Null);
#pragma warning restore 618
        }

        [Test]
        public void ListenerSource_DoesNotRegisterJointStatesPublisher()
        {
            PackageInfo package = PackageInfo.FindForAssembly(typeof(RobotJointStatesListener).Assembly);
            string sourcePath = Path.Combine(package.resolvedPath, "Runtime/Robots/ROS-Industrial Robots/RobotJointStatesListener.cs");

            string source = File.ReadAllText(sourcePath);
            Assert.That(source, Does.Not.Contain("RegisterPublisher<JointStateMsg>"));
            Assert.That(source, Does.Not.Contain(".Unsubscribe("));
        }

        private static JointStateMsg CreateMalformedMessage(string malformedField)
        {
            var message = new JointStateMsg
            {
                name = new[] { "joint_1", "joint_2" },
                position = new[] { 0.1d, 0.2d },
                velocity = System.Array.Empty<double>(),
                effort = System.Array.Empty<double>()
            };

            switch (malformedField)
            {
                case "missing":
                    message.name = new[] { "joint_1" };
                    message.position = new[] { 0.1d };
                    break;
                case "duplicate":
                    message.name = new[] { "joint_1", "joint_1" };
                    break;
                case "position-length":
                    message.position = new[] { 0.1d };
                    break;
                case "velocity-length":
                    message.velocity = new[] { 1d };
                    break;
                case "effort-length":
                    message.effort = new[] { 1d };
                    break;
                case "null-names":
                    message.name = null;
                    break;
                case "null-position":
                    message.position = null;
                    break;
                case "null-velocity":
                    message.velocity = null;
                    break;
                case "null-effort":
                    message.effort = null;
                    break;
                case "empty-name":
                    message.name = new[] { "joint_1", "" };
                    break;
                case "position-nan":
                    message.position = new[] { double.NaN, 0.2d };
                    break;
                case "position-positive-infinity":
                    message.position = new[] { double.PositiveInfinity, 0.2d };
                    break;
                case "position-negative-infinity":
                    message.position = new[] { double.NegativeInfinity, 0.2d };
                    break;
                case "velocity-nan":
                    message.velocity = new[] { double.NaN, 2d };
                    break;
                case "effort-positive-infinity":
                    message.effort = new[] { double.PositiveInfinity, 4d };
                    break;
            }

            return message;
        }

        private RobotJointStatesListener CreateListener(
            string[] names,
            ArticulationJointType[] jointTypes,
            out UrdfRobotMotionController controller)
        {
            var robotObject = new GameObject("robot");
            robotObject.SetActive(false);
            gameObjects.Add(robotObject);
            robotObject.AddComponent<UrdfRobot>();
            var robot = robotObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            robot.jointNames = new List<string>(names);
            robot.jointArticulationBodies = new List<ArticulationBody>();

            for (int i = 0; i < names.Length; i++)
            {
                var jointObject = new GameObject(names[i]);
                gameObjects.Add(jointObject);
                ArticulationBody body = jointObject.AddComponent<ArticulationBody>();
                body.jointType = jointTypes[i];
                robot.jointArticulationBodies.Add(body);
            }

            controller = robotObject.AddComponent<UrdfRobotMotionController>();
            controller.robot = robot;
            controller.jointTargets = new float[names.Length];

            var listener = robotObject.AddComponent<RobotJointStatesListener>();
            listener.unityRobot = robot;
            FieldInfo controllerField = typeof(RobotJointStatesListener).GetField("motionController");
            Assert.That(controllerField, Is.Not.Null, "The listener must expose its target motion controller.");
            controllerField.SetValue(listener, controller);
            return listener;
        }

        private ROSConnection SubscribeListener(RobotJointStatesListener listener)
        {
            var connectionObject = new GameObject("connection");
            gameObjects.Add(connectionObject);
            var connection = connectionObject.AddComponent<ROSConnection>();
            connection.ConnectOnStart = false;
            connection.listenForTFMessages = false;
            listener.rosConnection = connection;
            MethodInfo start = typeof(RobotJointStatesListener).GetMethod("Start", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(start, Is.Not.Null);
            start.Invoke(listener, null);
            return connection;
        }

        private static void DeliverSubscribedMessage(ROSConnection connection, string topicName, JointStateMsg message)
        {
            RosTopicState topic = connection.GetTopic(topicName);
            Assert.That(topic, Is.Not.Null);
            FieldInfo callbacksField = typeof(RosTopicState).GetField("m_SubscriberCallbacks", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(callbacksField, Is.Not.Null);
            var callbacks = (IList)callbacksField.GetValue(topic);
            Assert.That(callbacks, Has.Count.EqualTo(1));
            GetSubscribedCallback(connection, topicName).DynamicInvoke(message);
        }

        private static System.Delegate GetSubscribedCallback(ROSConnection connection, string topicName)
        {
            RosTopicState topic = connection.GetTopic(topicName);
            Assert.That(topic, Is.Not.Null);
            FieldInfo callbacksField = typeof(RosTopicState).GetField("m_SubscriberCallbacks", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(callbacksField, Is.Not.Null);
            var callbacks = (IList)callbacksField.GetValue(topic);
            Assert.That(callbacks, Has.Count.EqualTo(1));
            object registration = callbacks[0];
            FieldInfo callbackField = registration.GetType().GetField("Callback", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(callbackField, Is.Not.Null);
            return (System.Delegate)callbackField.GetValue(registration);
        }

        private static bool DelegateGraphDirectlyReferences(System.Delegate callback, object target)
        {
            if (ReferenceEquals(callback.Target, target)) { return true; }
            if (callback.Target == null) { return false; }

            foreach (FieldInfo field in callback.Target.GetType().GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic))
            {
                object value = field.GetValue(callback.Target);
                if (ReferenceEquals(value, target)) { return true; }
                if (value is System.Delegate nested && DelegateGraphDirectlyReferences(nested, target)) { return true; }
            }

            return false;
        }

        private static void AssertRejectedWithoutPublishing(
            RobotJointStatesListener listener,
            JointStateMsg message)
        {
            LogAssert.Expect(LogType.Warning, new Regex("^Rejected JointState message"));
            InvokeUpdate(listener, message);
            Assert.That(listener.jointNames, Is.Empty);
            Assert.That(listener.rosJointPositions, Is.Empty);
            Assert.That(listener.unityJointPositions, Is.Empty);
            Assert.That(listener.jointVelocities, Is.Empty);
            Assert.That(listener.jointEfforts, Is.Empty);
        }

        private static void InvokeUpdate(RobotJointStatesListener listener, JointStateMsg message)
        {
            MethodInfo method = typeof(RobotJointStatesListener).GetMethod("UpdateJointStates", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(method, Is.Not.Null);
            method.Invoke(listener, new object[] { message });
        }
    }
}
