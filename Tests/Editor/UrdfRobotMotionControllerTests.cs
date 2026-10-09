namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Tests
{
    using System.Collections.Generic;
    using NUnit.Framework;
    using Unity.Robotics.UrdfImporter;
    using UnityEngine;
    using UnityEngine.TestTools;

    public class UrdfRobotMotionControllerTests
    {
        private GameObject robotObject;

        [TearDown]
        public void TearDown()
        {
            if (robotObject != null) { Object.DestroyImmediate(robotObject); }
        }

        [Test]
        public void SetJointTargets_NullArrays_ThrowWithoutChangingTargets()
        {
            UrdfRobotMotionController controller = CreateController();

            Assert.Throws<System.ArgumentNullException>(() =>
                controller.SetJointTargets(null, new[] { 30d }));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
            Assert.Throws<System.ArgumentNullException>(() =>
                controller.SetJointTargets(new[] { "joint_2" }, null));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
        }

        [Test]
        public void SetJointTargets_NullOrEmptyNames_ThrowWithoutChangingTargets()
        {
            UrdfRobotMotionController controller = CreateController();

            foreach (string invalidName in new string[] { null, "" })
            {
                Assert.Throws<System.ArgumentException>(() =>
                    controller.SetJointTargets(new[] { "joint_1", invalidName }, new[] { 30d, 40d }));
                Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
            }
        }

        [Test]
        public void SetJointTargets_DuplicateName_ThrowsWithoutChangingTargets()
        {
            UrdfRobotMotionController controller = CreateController();

            Assert.Throws<System.ArgumentException>(() => controller.SetJointTargets(
                new[] { "joint_1", "joint_1" },
                new[] { 30d, 40d }));

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
        }

        [Test]
        public void SetJointTargets_UnknownName_ThrowsWithoutChangingTargets()
        {
            UrdfRobotMotionController controller = CreateController();

            System.ArgumentException exception = Assert.Throws<System.ArgumentException>(() =>
                controller.SetJointTargets(
                    new[] { "joint_1", "unknown_joint" },
                    new[] { 30d, 40d }));

            Assert.That(exception.Message, Does.Contain("unknown_joint"));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
        }

        [Test]
        public void SetJointTargets_NonFinitePosition_ThrowsWithoutChangingTargets()
        {
            UrdfRobotMotionController controller = CreateController();

            foreach (double invalidPosition in new[] { double.NaN, double.PositiveInfinity, double.NegativeInfinity })
            {
                Assert.Throws<System.ArgumentException>(() => controller.SetJointTargets(
                    new[] { "joint_1", "joint_2" },
                    new[] { 30d, invalidPosition }));
                Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
            }
        }

        [Test]
        public void SetJointTargets_MismatchedLengths_ThrowWithoutChangingTargets()
        {
            UrdfRobotMotionController controller = CreateController();

            Assert.Throws<System.ArgumentException>(() =>
                controller.SetJointTargets(new[] { "joint_1", "joint_2" }, new[] { 30d }));

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
        }

        [Test]
        public void SetJointTargets_InconsistentRobotJointMetadata_ThrowsArgumentException()
        {
            UrdfRobotMotionController controller = CreateController();
            controller.robot.jointArticulationBodies.RemoveAt(1);

            System.ArgumentException exception = Assert.Throws<System.ArgumentException>(() =>
                controller.SetJointTargets(new[] { "joint_2" }, new[] { 30d }));

            Assert.That(exception.Message, Does.Contain("joint names").And.Contain("articulation bodies"));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
        }

        [Test]
        public void SetJointTargets_PartialNamedUpdate_PreservesOtherTargets()
        {
            UrdfRobotMotionController controller = CreateController();

            controller.SetJointTargets(new[] { "joint_2" }, new[] { 30d });

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 30f }));
        }

        [Test]
        public void SetJointTargets_FiniteDoubleOutsideFloatRange_ThrowsAtomically()
        {
            UrdfRobotMotionController controller = CreateController();
            float[] originalTargets = controller.jointTargets;

            Assert.Throws<System.ArgumentException>(() => controller.SetJointTargets(
                new[] { "joint_1" }, new[] { double.MaxValue }));

            Assert.That(controller.jointTargets, Is.SameAs(originalTargets));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
        }

        [Test]
        public void SetJointTargets_PartialUpdateRequiresValidBaselineButCompleteUpdateRebuilds()
        {
            UrdfRobotMotionController controller = CreateController();
            foreach (float[] invalidBaseline in new[]
                     {
                         new[] { 10f },
                         new[] { 10f, float.PositiveInfinity },
                         null
                     })
            {
                controller.jointTargets = invalidBaseline;
                Assert.Throws<System.ArgumentException>(() =>
                    controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d }));
                Assert.That(controller.jointTargets, Is.SameAs(invalidBaseline));
            }

            controller.jointTargets = null;
            controller.SetJointTargets(
                new[] { "joint_2", "joint_1" },
                new[] { 40d, 30d });

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 30f, 40f }));
        }

        [Test]
        public void Move_FiniteDoubleOutsideFloatRange_ReturnsFalseWithOneErrorAndPreservesTargets()
        {
            UrdfRobotMotionController controller = CreateController();
            float[] originalTargets = controller.jointTargets;
            LogAssert.Expect(
                LogType.Error,
                new System.Text.RegularExpressions.Regex(
                    "^Caught exception when trying to move the robot: Joint target for 'joint_1' cannot be represented as a finite float\\.\\s*Parameter name: positions$"));

            object moveTask = typeof(UrdfRobotMotionController).GetMethod("Move").Invoke(
                controller,
                new object[]
                {
                    new[] { "joint_1" },
                    new[] { double.MaxValue },
                    default(System.Threading.CancellationToken)
                });
            object awaiter = moveTask.GetType().GetMethod("GetAwaiter").Invoke(moveTask, null);
            bool result = (bool)awaiter.GetType().GetMethod("GetResult").Invoke(awaiter, null);

            Assert.That(result, Is.False);
            Assert.That(controller.jointTargets, Is.SameAs(originalTargets));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 20f }));
        }

        [Test]
        public void InvalidRobotMetadata_IsRejectedByControllerAdmission()
        {
            UrdfRobotMotionController controller = CreateController();
            UrdfRobotKinematicsDataProvider robot = controller.robot;

            controller.robot = null;
            Assert.Throws<System.ArgumentNullException>(() => controller.SetJointTargets(
                System.Array.Empty<string>(), System.Array.Empty<double>()));
            controller.robot = robot;

            robot.jointNames = null;
            Assert.Throws<System.ArgumentException>(() => controller.SetJointTargets(
                System.Array.Empty<string>(), System.Array.Empty<double>()));
            robot.jointNames = new List<string> { "joint_1", "joint_2" };

            robot.jointArticulationBodies = null;
            Assert.Throws<System.ArgumentException>(() => controller.SetJointTargets(
                System.Array.Empty<string>(), System.Array.Empty<double>()));
            ArticulationBody body = robotObject.GetComponent<ArticulationBody>();
            robot.jointArticulationBodies = new List<ArticulationBody> { body, body };

            foreach (string invalidName in new string[] { null, "" })
            {
                robot.jointNames[0] = invalidName;
                Assert.Throws<System.ArgumentException>(() => controller.SetJointTargets(
                    System.Array.Empty<string>(), System.Array.Empty<double>()));
            }
            robot.jointNames[0] = "joint_1";

            robot.jointNames[1] = "joint_1";
            Assert.Throws<System.ArgumentException>(() => controller.SetJointTargets(
                System.Array.Empty<string>(), System.Array.Empty<double>()));
            robot.jointNames[1] = "joint_2";

            robot.jointArticulationBodies[1] = null;
            Assert.Throws<System.ArgumentException>(() => controller.SetJointTargets(
                System.Array.Empty<string>(), System.Array.Empty<double>()));
        }

        [Test]
        public void SetJointTargets_DuplicateLiveArticulationBody_ThrowsWithoutChangingTargets()
        {
            UrdfRobotMotionController controller = CreateController();
            controller.robot.jointArticulationBodies[1] = controller.robot.jointArticulationBodies[0];
            float[] originalTargets = controller.jointTargets;

            System.ArgumentException exception = Assert.Throws<System.ArgumentException>(() =>
                controller.SetJointTargets(
                    new[] { "joint_1", "joint_2" },
                    new[] { 30d, 40d }));

            Assert.That(exception.Message, Does.Contain("duplicate").And.Contain("articulation body"));
            Assert.That(controller.jointTargets, Is.SameAs(originalTargets));
        }

        [Test]
        public void KinematicFixedUpdate_CorruptedMetadataAfterAdmission_HasNoSideEffects()
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody baseBody, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Kinematic;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d });
            Quaternion baseRotation = baseBody.transform.rotation;
            Quaternion jointRotation = jointBody.transform.rotation;
            bool baseEnabled = baseBody.enabled;
            bool jointEnabled = jointBody.enabled;
            controller.robot.jointNames[0] = null;
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            Assert.That(baseBody.enabled, Is.EqualTo(baseEnabled));
            Assert.That(jointBody.enabled, Is.EqualTo(jointEnabled));
            Assert.That(baseBody.transform.rotation, Is.EqualTo(baseRotation));
            Assert.That(jointBody.transform.rotation, Is.EqualTo(jointRotation));
        }

        [Test]
        public void PhysicalFixedUpdate_CorruptedMetadataAfterAdmission_HasNoSideEffects()
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody _, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d });
            jointBody.enabled = false;
            ArticulationDrive originalDrive = jointBody.xDrive;
            float originalFriction = jointBody.jointFriction;
            float originalDamping = jointBody.angularDamping;
            controller.robot.jointNames[0] = "";
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            Assert.That(jointBody.enabled, Is.False);
            AssertDriveEqual(jointBody.xDrive, originalDrive);
            Assert.That(jointBody.jointFriction, Is.EqualTo(originalFriction));
            Assert.That(jointBody.angularDamping, Is.EqualTo(originalDamping));
        }

        [Test]
        public void FixedUpdate_ExternallyMutatedInfiniteTarget_HasNoSideEffects()
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody _, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d });
            controller.jointTargets[0] = float.PositiveInfinity;
            jointBody.enabled = false;
            ArticulationDrive originalDrive = jointBody.xDrive;
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            Assert.That(jointBody.enabled, Is.False);
            AssertDriveEqual(jointBody.xDrive, originalDrive);
        }

        [TestCase(UrdfRobotMotionController.ControlMode.Kinematic)]
        [TestCase(UrdfRobotMotionController.ControlMode.Physical)]
        public void FixedUpdate_ReorderedValidMetadata_HasNoEffectsAndRestorationDoesNotReviveAuthority(
            UrdfRobotMotionController.ControlMode mode)
        {
            UrdfRobotMotionController controller = CreateTwoJointFixedUpdateController(out ArticulationBody[] bodies);
            controller.controlMode = mode;
            foreach (ArticulationBody body in bodies) { body.enabled = false; }
            bool[] enabled = CaptureEnabled(bodies);
            Quaternion[] rotations = CaptureRotations(bodies);
            ArticulationDrive[] drives = CaptureDrives(bodies);
            List<string> names = controller.robot.jointNames;
            List<ArticulationBody> jointBodies = controller.robot.jointArticulationBodies;
            Swap(names, 0, 1);
            Swap(jointBodies, 0, 1);
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            AssertNoArticulationEffects(bodies, enabled, rotations, drives);

            Swap(names, 0, 1);
            Swap(jointBodies, 0, 1);
            controller.jointTargets = (float[])controller.jointTargets.Clone();
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            AssertNoArticulationEffects(bodies, enabled, rotations, drives);
        }

        [TestCase(UrdfRobotMotionController.ControlMode.Kinematic)]
        [TestCase(UrdfRobotMotionController.ControlMode.Physical)]
        public void FixedUpdate_SameNameBodySubstitution_HasNoEffectsAndInvalidatesAuthority(
            UrdfRobotMotionController.ControlMode mode)
        {
            UrdfRobotMotionController controller = CreateTwoJointFixedUpdateController(out ArticulationBody[] originalBodies);
            controller.controlMode = mode;
            var replacementObject = new GameObject("joint_1_replacement");
            replacementObject.transform.SetParent(robotObject.transform, false);
            ArticulationBody replacement = replacementObject.AddComponent<ArticulationBody>();
            replacement.jointType = ArticulationJointType.RevoluteJoint;
            controller.robot.jointArticulationBodies[0] = replacement;
            controller.robot.articulationBodies.Add(replacement);
            var observedBodies = new[] { originalBodies[0], originalBodies[1], originalBodies[2], replacement };
            foreach (ArticulationBody body in observedBodies) { body.enabled = false; }
            bool[] enabled = CaptureEnabled(observedBodies);
            Quaternion[] rotations = CaptureRotations(observedBodies);
            ArticulationDrive[] drives = CaptureDrives(observedBodies);
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            AssertNoArticulationEffects(observedBodies, enabled, rotations, drives);
        }

        [Test]
        public void SetJointTargets_AfterObservedMismatch_PartialCannotReadmitButCompleteCan()
        {
            UrdfRobotMotionController controller = CreateTwoJointFixedUpdateController(out ArticulationBody[] bodies);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            Swap(controller.robot.jointNames, 0, 1);
            Swap(controller.robot.jointArticulationBodies, 0, 1);

            Assert.Throws<System.ArgumentException>(() =>
                controller.SetJointTargets(new[] { "joint_2" }, new[] { 50d }));

            Swap(controller.robot.jointNames, 0, 1);
            Swap(controller.robot.jointArticulationBodies, 0, 1);
            Assert.Throws<System.ArgumentException>(() =>
                controller.SetJointTargets(new[] { "joint_1" }, new[] { 60d }));

            controller.SetJointTargets(
                new[] { "joint_2", "joint_1" },
                new[] { 22d, 11d });
            InvokeFixedUpdate(controller);

            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 11f, 22f }));
            Assert.That(bodies[1].xDrive.target, Is.EqualTo(11f));
            Assert.That(bodies[2].xDrive.target, Is.EqualTo(22f));
        }

        [Test]
        public void SetJointTargets_MalformedRequestUnderUnchangedMetadata_PreservesBinding()
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody _, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d });
            float[] originalTargets = controller.jointTargets;

            Assert.Throws<System.ArgumentNullException>(() =>
                controller.SetJointTargets(null, new[] { 40d }));
            Assert.That(controller.jointTargets, Is.SameAs(originalTargets));

            jointBody.enabled = false;
            InvokeFixedUpdate(controller);

            Assert.That(jointBody.enabled, Is.True);
            Assert.That(jointBody.xDrive.target, Is.EqualTo(30f));
        }

        [Test]
        public void FixedUpdate_InvalidPublicTargetsThenRepair_DoesNotReviveAuthority()
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody _, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d });
            controller.jointTargets[0] = float.NaN;
            jointBody.enabled = false;
            ArticulationDrive originalDrive = jointBody.xDrive;
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            Assert.That(jointBody.enabled, Is.False);
            AssertDriveEqual(jointBody.xDrive, originalDrive);

            controller.jointTargets = new[] { 40f };
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));
            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            Assert.That(jointBody.enabled, Is.False);
            AssertDriveEqual(jointBody.xDrive, originalDrive);
        }

        [Test]
        public void FixedUpdate_FinitePublicTargetMutationAndReplacementRemainAccepted()
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody _, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            controller.jointTargets[0] = 12f;
            jointBody.enabled = false;

            InvokeFixedUpdate(controller);

            Assert.That(jointBody.enabled, Is.True);
            Assert.That(jointBody.xDrive.target, Is.EqualTo(12f));

            controller.jointTargets = new[] { 34f };
            jointBody.enabled = false;
            InvokeFixedUpdate(controller);

            Assert.That(jointBody.enabled, Is.True);
            Assert.That(jointBody.xDrive.target, Is.EqualTo(34f));
        }

        [Test]
        public void FixedUpdate_ReorderedInputNamesApplyToCorrectBodies()
        {
            UrdfRobotMotionController controller = CreateTwoJointFixedUpdateController(out ArticulationBody[] bodies);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            controller.SetJointTargets(
                new[] { "joint_2", "joint_1" },
                new[] { 42d, 31d });

            InvokeFixedUpdate(controller);

            Assert.That(bodies[1].xDrive.target, Is.EqualTo(31f));
            Assert.That(bodies[2].xDrive.target, Is.EqualTo(42f));
        }

        [Test]
        public void OnEnable_AfterInvalidationInitializesFreshZeroBinding()
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody _, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d });
            float[] admittedTargets = controller.jointTargets;
            controller.jointTargets[0] = float.NaN;
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));
            InvokeFixedUpdate(controller);

            controller.OnEnable();

            Assert.That(controller.jointTargets, Is.Not.SameAs(admittedTargets));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 0f }));
            jointBody.enabled = false;
            InvokeFixedUpdate(controller);
            Assert.That(jointBody.enabled, Is.True);
            Assert.That(jointBody.xDrive.target, Is.EqualTo(0f));
        }

        [TestCase(ArticulationJointType.PrismaticJoint)]
        [TestCase(ArticulationJointType.SphericalJoint)]
        [TestCase(ArticulationJointType.FixedJoint)]
        public void KinematicFixedUpdate_MixedUnsupportedJointType_HasNoEffects(
            ArticulationJointType unsupportedType)
        {
            UrdfRobotMotionController controller = CreateTwoJointFixedUpdateController(out ArticulationBody[] bodies);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Kinematic;
            bodies[2].jointType = unsupportedType;
            foreach (ArticulationBody body in bodies) { body.enabled = true; }
            bool[] enabled = CaptureEnabled(bodies);
            Quaternion[] rotations = CaptureRotations(bodies);
            ArticulationDrive[] drives = CaptureDrives(bodies);
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            AssertNoArticulationEffects(bodies, enabled, rotations, drives);
        }

        [Test]
        public void KinematicFixedUpdate_NullAllBodyEntry_HasNoEffects()
        {
            UrdfRobotMotionController controller = CreateTwoJointFixedUpdateController(out ArticulationBody[] bodies);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Kinematic;
            controller.robot.articulationBodies.Add(null);
            bool[] enabled = CaptureEnabled(bodies);
            Quaternion[] rotations = CaptureRotations(bodies);
            ArticulationDrive[] drives = CaptureDrives(bodies);
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            AssertNoArticulationEffects(bodies, enabled, rotations, drives);
        }

        [Test]
        public void KinematicFixedUpdate_OmittedJointBody_HasNoEffectsAndInvalidatesBinding()
        {
            UrdfRobotMotionController controller = CreateKinematicIdentitySetController(out ArticulationBody[] bodies);
            controller.robot.articulationBodies.Remove(bodies[1]);

            AssertKinematicIdentitySetRejection(controller, bodies);
        }

        [Test]
        public void KinematicFixedUpdate_OmittedParentBody_HasNoEffectsAndInvalidatesBinding()
        {
            UrdfRobotMotionController controller = CreateKinematicIdentitySetController(out ArticulationBody[] bodies);
            controller.robot.articulationBodies.Remove(bodies[0]);

            AssertKinematicIdentitySetRejection(controller, bodies);
        }

        [Test]
        public void KinematicFixedUpdate_OmittedNonJointHierarchyBody_HasNoEffectsAndInvalidatesBinding()
        {
            UrdfRobotMotionController controller = CreateKinematicIdentitySetController(out ArticulationBody[] bodies);
            controller.robot.articulationBodies.Remove(bodies[2]);

            AssertKinematicIdentitySetRejection(controller, bodies);
        }

        [Test]
        public void KinematicFixedUpdate_DuplicateBodyEntry_HasNoEffectsAndInvalidatesBinding()
        {
            UrdfRobotMotionController controller = CreateKinematicIdentitySetController(out ArticulationBody[] bodies);
            int otherBodyIndex = controller.robot.articulationBodies.IndexOf(bodies[2]);
            controller.robot.articulationBodies[otherBodyIndex] = bodies[1];

            AssertKinematicIdentitySetRejection(controller, bodies);
        }

        [Test]
        public void KinematicFixedUpdate_ForeignLiveBody_HasNoEffectsAndInvalidatesBinding()
        {
            UrdfRobotMotionController controller = CreateKinematicIdentitySetController(out ArticulationBody[] bodies);
            var foreignObject = new GameObject("foreign_body");
            ArticulationBody foreignBody = foreignObject.AddComponent<ArticulationBody>();
            try
            {
                int otherBodyIndex = controller.robot.articulationBodies.IndexOf(bodies[2]);
                controller.robot.articulationBodies[otherBodyIndex] = foreignBody;
                var observedBodies = new[] { bodies[0], bodies[1], bodies[2], foreignBody };

                AssertKinematicIdentitySetRejection(controller, observedBodies);
            }
            finally
            {
                Object.DestroyImmediate(foreignObject);
            }
        }

        [Test]
        public void KinematicFixedUpdate_ReorderedCompleteBodySet_IsAccepted()
        {
            UrdfRobotMotionController controller = CreateKinematicIdentitySetController(out ArticulationBody[] bodies);
            controller.robot.articulationBodies.Reverse();
            ArticulationDrive[] drives = CaptureDrives(bodies);

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.True);
            foreach (ArticulationBody body in bodies) { Assert.That(body.enabled, Is.False); }
            for (int i = 0; i < bodies.Length; i++) { AssertDriveEqual(bodies[i].xDrive, drives[i]); }
            Assert.That(bodies[1].transform.rotation, Is.Not.EqualTo(Quaternion.identity));
        }

        [TestCase(ArticulationJointType.PrismaticJoint)]
        [TestCase(ArticulationJointType.SphericalJoint)]
        [TestCase(ArticulationJointType.FixedJoint)]
        public void PhysicalFixedUpdate_DoesNotImposeKinematicJointTypeRestriction(
            ArticulationJointType jointType)
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody _, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Physical;
            jointBody.jointType = jointType;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 25d });
            jointBody.enabled = false;

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.True);
            Assert.That(jointBody.enabled, Is.True);
            Assert.That(jointBody.xDrive.target, Is.EqualTo(25f));
        }

        [Test]
        public void StopMotion_UsesOneMetadataSnapshotInsteadOfPublicRecapturePath()
        {
            UnityEditor.PackageManager.PackageInfo package =
                UnityEditor.PackageManager.PackageInfo.FindForAssembly(typeof(UrdfRobotMotionController).Assembly);
            string sourcePath = System.IO.Path.Combine(
                package.resolvedPath,
                "Runtime/Robots/Unity Robots/UrdfRobotMotionController.cs");
            string source = System.IO.File.ReadAllText(sourcePath);
            int stopStart = source.IndexOf("public async UniTask<bool> StopMotion()", System.StringComparison.Ordinal);
            int moveStart = source.IndexOf("public async UniTask<bool> Move(", stopStart, System.StringComparison.Ordinal);
            Assert.That(stopStart, Is.GreaterThanOrEqualTo(0));
            Assert.That(moveStart, Is.GreaterThan(stopStart));
            string stopSource = source.Substring(stopStart, moveStart - stopStart);

            Assert.That(stopSource, Does.Not.Contain("GetJointPositions()"));
            Assert.That(stopSource, Does.Not.Contain("SetJointTargets("));
            Assert.That(stopSource, Does.Contain("MetadataMatches"));
            Assert.That(stopSource, Does.Contain("PublishTargets"));
        }

        [Test]
        public void StopMotion_ReorderedMetadataPublishesPositionsInCapturedOrder()
        {
            UrdfRobotMotionController controller = CreateTwoJointFixedUpdateController(out ArticulationBody[] bodies);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Kinematic;
            robotObject.SetActive(true);
            bodies[1].transform.rotation = Quaternion.Euler(10f, 0f, 0f);
            bodies[2].transform.rotation = Quaternion.Euler(20f, 0f, 0f);
            Swap(controller.robot.jointNames, 0, 1);
            Swap(controller.robot.jointArticulationBodies, 0, 1);
            (string[] capturedNames, double[] capturedPositions) = controller.GetJointPositions();

            typeof(UrdfRobotMotionController).GetMethod("StopMotion").Invoke(controller, null);

            Assert.That(capturedNames, Is.EqualTo(new[] { "joint_2", "joint_1" }));
            Assert.That(controller.jointTargets[0], Is.EqualTo((float)capturedPositions[0]).Within(0.001f));
            Assert.That(controller.jointTargets[1], Is.EqualTo((float)capturedPositions[1]).Within(0.001f));
        }

        [Test]
        public void StopMotion_RepublishesThroughJointTargetAdmission()
        {
            UrdfRobotMotionController controller = CreateController();
            controller.robot.jointNames.Clear();
            controller.robot.jointArticulationBodies.Clear();
            float[] invalidTargets = { float.NaN };
            controller.jointTargets = invalidTargets;

            typeof(UrdfRobotMotionController).GetMethod("StopMotion").Invoke(controller, null);

            Assert.That(controller.jointTargets, Is.Not.SameAs(invalidTargets));
            Assert.That(controller.jointTargets, Is.Empty);
        }

        [Test]
        public void KinematicFixedUpdate_InvalidRequiredBodyCollection_HasNoSideEffects()
        {
            UrdfRobotMotionController controller = CreateFixedUpdateController(out ArticulationBody baseBody, out ArticulationBody jointBody);
            controller.controlMode = UrdfRobotMotionController.ControlMode.Kinematic;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d });
            controller.robot.articulationBodies = null;
            bool baseEnabled = baseBody.enabled;
            bool jointEnabled = jointBody.enabled;
            Quaternion jointRotation = jointBody.transform.rotation;
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            Assert.That(baseBody.enabled, Is.EqualTo(baseEnabled));
            Assert.That(jointBody.enabled, Is.EqualTo(jointEnabled));
            Assert.That(jointBody.transform.rotation, Is.EqualTo(jointRotation));
        }

        [Test]
        public void Move_PartialValidRequestPreservesOtherTargetsAndPublishesFreshArray()
        {
            UrdfRobotMotionController controller = CreateController();
            float[] originalTargets = controller.jointTargets;

            object moveTask = typeof(UrdfRobotMotionController).GetMethod("Move").Invoke(
                controller,
                new object[]
                {
                    new[] { "joint_2" },
                    new[] { 30d },
                    default(System.Threading.CancellationToken)
                });
            object awaiter = moveTask.GetType().GetMethod("GetAwaiter").Invoke(moveTask, null);
            bool result = (bool)awaiter.GetType().GetMethod("GetResult").Invoke(awaiter, null);

            Assert.That(result, Is.True);
            Assert.That(controller.jointTargets, Is.Not.SameAs(originalTargets));
            Assert.That(controller.jointTargets, Is.EqualTo(new[] { 10f, 30f }));
        }

        private static void AssertKinematicIdentitySetRejection(
            UrdfRobotMotionController controller,
            ArticulationBody[] observedBodies)
        {
            bool[] enabled = CaptureEnabled(observedBodies);
            Quaternion[] rotations = CaptureRotations(observedBodies);
            ArticulationDrive[] drives = CaptureDrives(observedBodies);
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("^Rejected robot controller FixedUpdate"));

            InvokeFixedUpdate(controller);

            Assert.That(controller.enabled, Is.False);
            AssertNoArticulationEffects(observedBodies, enabled, rotations, drives);
            System.Reflection.FieldInfo binding = typeof(UrdfRobotMotionController).GetField(
                "targetMetadataBinding",
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
            Assert.That(binding, Is.Not.Null);
            Assert.That(binding.GetValue(controller), Is.Null);
        }

        private static void AssertDriveEqual(ArticulationDrive actual, ArticulationDrive expected)
        {
            Assert.That(actual.target, Is.EqualTo(expected.target));
            Assert.That(actual.forceLimit, Is.EqualTo(expected.forceLimit));
            Assert.That(actual.stiffness, Is.EqualTo(expected.stiffness));
            Assert.That(actual.damping, Is.EqualTo(expected.damping));
        }

        private static bool[] CaptureEnabled(ArticulationBody[] bodies)
        {
            var result = new bool[bodies.Length];
            for (int i = 0; i < bodies.Length; i++) { result[i] = bodies[i].enabled; }
            return result;
        }

        private static Quaternion[] CaptureRotations(ArticulationBody[] bodies)
        {
            var result = new Quaternion[bodies.Length];
            for (int i = 0; i < bodies.Length; i++) { result[i] = bodies[i].transform.rotation; }
            return result;
        }

        private static ArticulationDrive[] CaptureDrives(ArticulationBody[] bodies)
        {
            var result = new ArticulationDrive[bodies.Length];
            for (int i = 0; i < bodies.Length; i++) { result[i] = bodies[i].xDrive; }
            return result;
        }

        private static void AssertNoArticulationEffects(
            ArticulationBody[] bodies,
            bool[] enabled,
            Quaternion[] rotations,
            ArticulationDrive[] drives)
        {
            for (int i = 0; i < bodies.Length; i++)
            {
                Assert.That(bodies[i].enabled, Is.EqualTo(enabled[i]));
                Assert.That(bodies[i].transform.rotation, Is.EqualTo(rotations[i]));
                AssertDriveEqual(bodies[i].xDrive, drives[i]);
            }
        }

        private static void Swap<T>(List<T> values, int first, int second)
        {
            T value = values[first];
            values[first] = values[second];
            values[second] = value;
        }

        private static void InvokeFixedUpdate(UrdfRobotMotionController controller)
        {
            System.Reflection.MethodInfo method = typeof(UrdfRobotMotionController).GetMethod(
                "FixedUpdate",
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
            Assert.That(method, Is.Not.Null);
            method.Invoke(controller, null);
        }

        private UrdfRobotMotionController CreateKinematicIdentitySetController(out ArticulationBody[] bodies)
        {
            robotObject = new GameObject("robot");
            robotObject.SetActive(false);
            UrdfRobot urdfRobot = robotObject.AddComponent<UrdfRobot>();
            var robot = robotObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            robot.robot = urdfRobot;
            ArticulationBody baseBody = robotObject.AddComponent<ArticulationBody>();
            var jointObject = new GameObject("joint_1");
            jointObject.transform.SetParent(robotObject.transform, false);
            ArticulationBody jointBody = jointObject.AddComponent<ArticulationBody>();
            jointBody.jointType = ArticulationJointType.RevoluteJoint;
            var otherObject = new GameObject("other_body");
            otherObject.transform.SetParent(robotObject.transform, false);
            ArticulationBody otherBody = otherObject.AddComponent<ArticulationBody>();
            var controller = robotObject.AddComponent<UrdfRobotMotionController>();
            controller.robot = robot;
            controller.controlMode = UrdfRobotMotionController.ControlMode.Kinematic;
            robotObject.SetActive(true);
            robot.jointNames = new List<string> { "joint_1" };
            robot.jointArticulationBodies = new List<ArticulationBody> { jointBody };
            robot.articulationBodies = new List<ArticulationBody>(
                robot.GetComponentsInChildren<ArticulationBody>());
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 30d });
            bodies = new[] { baseBody, jointBody, otherBody };
            return controller;
        }

        private UrdfRobotMotionController CreateFixedUpdateController(
            out ArticulationBody baseBody,
            out ArticulationBody jointBody)
        {
            robotObject = new GameObject("robot");
            robotObject.SetActive(false);
            robotObject.AddComponent<UrdfRobot>();
            var robot = robotObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            baseBody = robotObject.AddComponent<ArticulationBody>();
            var jointObject = new GameObject("joint_1");
            jointObject.transform.SetParent(robotObject.transform, false);
            jointBody = jointObject.AddComponent<ArticulationBody>();
            jointBody.jointType = ArticulationJointType.RevoluteJoint;
            robot.jointNames = new List<string> { "joint_1" };
            robot.jointArticulationBodies = new List<ArticulationBody> { jointBody };
            robot.articulationBodies = new List<ArticulationBody> { baseBody, jointBody };
            var controller = robotObject.AddComponent<UrdfRobotMotionController>();
            controller.robot = robot;
            controller.SetJointTargets(new[] { "joint_1" }, new[] { 0d });
            return controller;
        }

        private UrdfRobotMotionController CreateTwoJointFixedUpdateController(out ArticulationBody[] bodies)
        {
            robotObject = new GameObject("robot");
            robotObject.SetActive(false);
            robotObject.AddComponent<UrdfRobot>();
            var robot = robotObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            ArticulationBody baseBody = robotObject.AddComponent<ArticulationBody>();
            var firstJointObject = new GameObject("joint_1");
            firstJointObject.transform.SetParent(robotObject.transform, false);
            ArticulationBody firstBody = firstJointObject.AddComponent<ArticulationBody>();
            firstBody.jointType = ArticulationJointType.RevoluteJoint;
            var secondJointObject = new GameObject("joint_2");
            secondJointObject.transform.SetParent(robotObject.transform, false);
            ArticulationBody secondBody = secondJointObject.AddComponent<ArticulationBody>();
            secondBody.jointType = ArticulationJointType.RevoluteJoint;
            robot.jointNames = new List<string> { "joint_1", "joint_2" };
            robot.jointArticulationBodies = new List<ArticulationBody> { firstBody, secondBody };
            robot.articulationBodies = new List<ArticulationBody> { baseBody, firstBody, secondBody };
            var controller = robotObject.AddComponent<UrdfRobotMotionController>();
            controller.robot = robot;
            controller.SetJointTargets(
                new[] { "joint_1", "joint_2" },
                new[] { 10d, 20d });
            bodies = new[] { baseBody, firstBody, secondBody };
            return controller;
        }

        private UrdfRobotMotionController CreateController()
        {
            robotObject = new GameObject("robot");
            robotObject.SetActive(false);
            robotObject.AddComponent<UrdfRobot>();
            var robot = robotObject.AddComponent<UrdfRobotKinematicsDataProvider>();
            robot.jointNames = new List<string> { "joint_1", "joint_2" };
            var firstJointObject = new GameObject("joint_1");
            firstJointObject.transform.SetParent(robotObject.transform, false);
            ArticulationBody firstBody = firstJointObject.AddComponent<ArticulationBody>();
            var secondJointObject = new GameObject("joint_2");
            secondJointObject.transform.SetParent(robotObject.transform, false);
            ArticulationBody secondBody = secondJointObject.AddComponent<ArticulationBody>();
            robot.jointArticulationBodies = new List<ArticulationBody> { firstBody, secondBody };
            var controller = robotObject.AddComponent<UrdfRobotMotionController>();
            controller.robot = robot;
            controller.SetJointTargets(
                new[] { "joint_1", "joint_2" },
                new[] { 10d, 20d });
            return controller;
        }
    }
}
