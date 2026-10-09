namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Tests
{
    using System.Collections.Generic;
    using NUnit.Framework;
    using Unity.Robotics.UrdfImporter;
    using UnityEngine;

    public class RobotJointPositionsConversionUtilityTests
    {
        private readonly List<GameObject> gameObjects = new List<GameObject>();

        [TearDown]
        public void TearDown()
        {
            foreach (GameObject gameObject in gameObjects) { Object.DestroyImmediate(gameObject); }
            gameObjects.Clear();
        }

        [Test]
        public void RosToUnity_NullInputVectors_ThrowArgumentNullException()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint });

            Assert.Throws<System.ArgumentNullException>(() =>
                RobotJointPositionsConversionUtility.RosToUnity(robot, null, new[] { 0.5d }));
            Assert.Throws<System.ArgumentNullException>(() =>
                RobotJointPositionsConversionUtility.RosToUnity(robot, new[] { "joint_1" }, null));
        }

        [Test]
        public void RosToUnity_ReorderedNames_MapsToRobotOrderAndConvertsJointUnits()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "revolute_joint", "prismatic_joint" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.PrismaticJoint });

            double[] result = RobotJointPositionsConversionUtility.RosToUnity(
                robot,
                new[] { "prismatic_joint", "revolute_joint" },
                new[] { 0.25d, System.Math.PI / 2d });

            Assert.That(result, Is.EqualTo(new[] { 90d, 0.25d }).Within(0.000001d));
        }

        [Test]
        public void RosToUnity_MissingRequiredJoint_ThrowsWithoutReturningPartialTargets()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.RevoluteJoint });

            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.RosToUnity(
                robot,
                new[] { "joint_1" },
                new[] { 0.5d }));
        }

        [Test]
        public void RosToUnity_EmptyRosName_Throws()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint });

            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.RosToUnity(
                robot,
                new[] { "joint_1", "" },
                new[] { 0.5d, 1d }));
        }

        [Test]
        public void RosToUnity_DuplicateRosNames_Throws()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint });

            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.RosToUnity(
                robot,
                new[] { "joint_1", "joint_1" },
                new[] { 0.5d, 1d }));
        }

        [Test]
        public void RosToUnity_NonFinitePositions_Throw()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint });

            foreach (double position in new[] { double.NaN, double.PositiveInfinity, double.NegativeInfinity })
            {
                Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.RosToUnity(
                    robot,
                    new[] { "joint_1" },
                    new[] { position }));
            }
        }

        [Test]
        public void RosToUnity_ConversionOverflow_ThrowsArgumentException()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint });

            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.RosToUnity(
                robot,
                new[] { "joint_1" },
                new[] { double.MaxValue }));
        }

        [Test]
        public void RosToUnity_InconsistentNameAndPositionLengths_ThrowsArgumentException()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.RevoluteJoint });

            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.RosToUnity(
                robot,
                new[] { "joint_1", "joint_2" },
                new[] { 0.5d }));
        }

        [Test]
        public void RosToUnity_InconsistentRobotJointMetadata_ThrowsArgumentException()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.RevoluteJoint });
            robot.jointArticulationBodies.RemoveAt(1);

            System.ArgumentException exception = Assert.Throws<System.ArgumentException>(() =>
                RobotJointPositionsConversionUtility.RosToUnity(
                    robot,
                    new[] { "joint_1", "joint_2" },
                    new[] { 0.5d, 1d }));
            Assert.That(exception.Message, Does.Contain("joint names").And.Contain("articulation bodies"));
        }

        [Test]
        public void UnityToRos_NullInputVectors_ThrowArgumentNullException()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint });

            Assert.Throws<System.ArgumentNullException>(() =>
                RobotJointPositionsConversionUtility.UnityToRos(robot, null, new[] { 30d }));
            Assert.Throws<System.ArgumentNullException>(() =>
                RobotJointPositionsConversionUtility.UnityToRos(robot, new[] { "joint_1" }, null));
        }

        [Test]
        public void UnityToRos_NullOrEmptyNames_ThrowForNamesArgument()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint });

            foreach (string invalidName in new string[] { null, "" })
            {
                System.ArgumentException exception = Assert.Throws<System.ArgumentException>(() =>
                    RobotJointPositionsConversionUtility.UnityToRos(
                        robot,
                        new[] { invalidName },
                        new[] { 30d }));
                Assert.That(exception.ParamName, Is.EqualTo("names"));
            }
        }

        [Test]
        public void UnityToRos_NonFinitePositions_Throw()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1" },
                new[] { ArticulationJointType.RevoluteJoint });

            foreach (double position in new[] { double.NaN, double.PositiveInfinity, double.NegativeInfinity })
            {
                Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                    robot,
                    new[] { "joint_1" },
                    new[] { position }));
            }
        }

        [Test]
        public void UnityToRos_InconsistentRobotJointMetadata_ThrowsArgumentException()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.RevoluteJoint });
            robot.jointArticulationBodies.RemoveAt(1);

            System.ArgumentException exception = Assert.Throws<System.ArgumentException>(() =>
                RobotJointPositionsConversionUtility.UnityToRos(
                    robot,
                    new[] { "joint_2" },
                    new[] { 30d }));
            Assert.That(exception.Message, Does.Contain("joint names").And.Contain("articulation bodies"));
        }

        [Test]
        public void UnityToRos_ReorderedNames_ConvertsByExactNameInRequestedOrder()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "revolute_joint", "prismatic_joint" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.PrismaticJoint });

            double[] result = RobotJointPositionsConversionUtility.UnityToRos(
                robot,
                new[] { "prismatic_joint", "revolute_joint" },
                new[] { 0.25d, 90d });

            Assert.That(result, Is.EqualTo(new[] { 0.25d, System.Math.PI / 2d }).Within(0.000001d));
        }

        [Test]
        public void InvalidRobotMetadata_IsRejectedByConversionEntryPoints()
        {
            Assert.Throws<System.ArgumentNullException>(() =>
                RobotJointPositionsConversionUtility.UnityToRos(null, System.Array.Empty<string>(), System.Array.Empty<double>()));

            UrdfRobotKinematicsDataProvider nullNames = CreateRobot(
                new[] { "joint_1" }, new[] { ArticulationJointType.RevoluteJoint });
            nullNames.jointNames = null;
            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                nullNames, new[] { "joint_1" }, new[] { 1d }));

            UrdfRobotKinematicsDataProvider nullBodies = CreateRobot(
                new[] { "joint_1" }, new[] { ArticulationJointType.RevoluteJoint });
            nullBodies.jointArticulationBodies = null;
            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                nullBodies, new[] { "joint_1" }, new[] { 1d }));

            foreach (string invalidName in new string[] { null, "" })
            {
                UrdfRobotKinematicsDataProvider invalidNames = CreateRobot(
                    new[] { "joint_1" }, new[] { ArticulationJointType.RevoluteJoint });
                invalidNames.jointNames[0] = invalidName;
                Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                    invalidNames, System.Array.Empty<string>(), System.Array.Empty<double>()));
            }

            UrdfRobotKinematicsDataProvider duplicateNames = CreateRobot(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.RevoluteJoint });
            duplicateNames.jointNames[1] = "joint_1";
            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                duplicateNames, System.Array.Empty<string>(), System.Array.Empty<double>()));

            UrdfRobotKinematicsDataProvider nullBody = CreateRobot(
                new[] { "joint_1" }, new[] { ArticulationJointType.RevoluteJoint });
            nullBody.jointArticulationBodies[0] = null;
            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                nullBody, new[] { "joint_1" }, new[] { 1d }));

            UrdfRobotKinematicsDataProvider destroyedBody = CreateRobot(
                new[] { "joint_1" }, new[] { ArticulationJointType.RevoluteJoint });
            Object.DestroyImmediate(destroyedBody.jointArticulationBodies[0]);
            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                destroyedBody, new[] { "joint_1" }, new[] { 1d }));
        }

        [Test]
        public void DuplicateLiveArticulationBody_IsRejectedByConversionEntryPoints()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.RevoluteJoint });
            robot.jointArticulationBodies[1] = robot.jointArticulationBodies[0];

            System.ArgumentException exception = Assert.Throws<System.ArgumentException>(() =>
                RobotJointPositionsConversionUtility.UnityToRos(
                    robot,
                    new[] { "joint_1", "joint_2" },
                    new[] { 1d, 2d }));

            Assert.That(exception.Message, Does.Contain("duplicate").And.Contain("articulation body"));
        }

        [Test]
        public void NamedVectorContract_CoversPartialUnknownDuplicateLengthAndExtraRosNames()
        {
            UrdfRobotKinematicsDataProvider robot = CreateRobot(
                new[] { "joint_1", "joint_2" },
                new[] { ArticulationJointType.RevoluteJoint, ArticulationJointType.PrismaticJoint });

            double[] partial = RobotJointPositionsConversionUtility.UnityToRos(
                robot, new[] { "joint_2" }, new[] { 0.25d });
            Assert.That(partial, Is.EqualTo(new[] { 0.25d }));

            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                robot, new[] { "joint_1", "joint_2" }, new[] { 1d }));
            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                robot, new[] { "joint_1", "joint_1" }, new[] { 1d, 2d }));
            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.UnityToRos(
                robot, new[] { "unknown" }, new[] { 1d }));

            double[] mapped = RobotJointPositionsConversionUtility.MapToRobotOrder(
                robot,
                new[] { "extra", "joint_2", "joint_1" },
                new[] { 9d, 2d, 1d },
                "position");
            Assert.That(mapped, Is.EqualTo(new[] { 1d, 2d }));
            Assert.Throws<System.ArgumentException>(() => RobotJointPositionsConversionUtility.MapToRobotOrder(
                robot,
                new[] { "extra", "joint_2", "joint_1" },
                new[] { double.NaN, 2d, 1d },
                "position"));
        }

        private UrdfRobotKinematicsDataProvider CreateRobot(string[] names, ArticulationJointType[] jointTypes)
        {
            var robotObject = new GameObject("robot");
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

            return robot;
        }
    }
}
