namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using System;
    using System.Collections.Generic;
    using UnityEngine;

    internal static class RobotJointContract
    {
        internal sealed class Metadata
        {
            internal Metadata(
                UrdfRobotKinematicsDataProvider provider,
                string[] names,
                ArticulationBody[] bodies,
                ArticulationJointType[] jointTypes,
                Dictionary<string, int> indicesByName)
            {
                Provider = provider;
                Names = names;
                Bodies = bodies;
                JointTypes = jointTypes;
                IndicesByName = indicesByName;
            }

            internal UrdfRobotKinematicsDataProvider Provider { get; }
            internal string[] Names { get; }
            internal ArticulationBody[] Bodies { get; }
            internal ArticulationJointType[] JointTypes { get; }
            internal Dictionary<string, int> IndicesByName { get; }
            internal int Count => Names.Length;
        }

        internal sealed class NamedVector
        {
            internal NamedVector(int[] robotIndices, double[] values, bool isComplete)
            {
                RobotIndices = robotIndices;
                Values = values;
                IsComplete = isComplete;
            }

            internal int[] RobotIndices { get; }
            internal double[] Values { get; }
            internal bool IsComplete { get; }
        }

        internal static Metadata Capture(
            UrdfRobotKinematicsDataProvider provider,
            string parameterName)
        {
            if (provider == null) { throw new ArgumentNullException(parameterName); }
            if (provider.jointNames == null)
            {
                throw new ArgumentException("Robot joint names collection cannot be null.", parameterName);
            }

            if (provider.jointArticulationBodies == null)
            {
                throw new ArgumentException("Robot joint articulation bodies collection cannot be null.", parameterName);
            }

            if (provider.jointNames.Count != provider.jointArticulationBodies.Count)
            {
                throw new ArgumentException(
                    "Robot joint names and articulation bodies must have the same count.",
                    parameterName);
            }

            var names = new string[provider.jointNames.Count];
            var bodies = new ArticulationBody[provider.jointArticulationBodies.Count];
            var jointTypes = new ArticulationJointType[provider.jointArticulationBodies.Count];
            var indicesByName = new Dictionary<string, int>(names.Length, StringComparer.Ordinal);
            for (int i = 0; i < names.Length; i++)
            {
                string name = provider.jointNames[i];
                if (string.IsNullOrEmpty(name))
                {
                    throw new ArgumentException("Robot joint names cannot contain null or empty entries.", parameterName);
                }

                if (indicesByName.ContainsKey(name))
                {
                    throw new ArgumentException($"Robot contains duplicate joint name '{name}'.", parameterName);
                }

                indicesByName.Add(name, i);

                ArticulationBody body = provider.jointArticulationBodies[i];
                if (body == null)
                {
                    throw new ArgumentException($"Robot joint '{name}' has a null or destroyed articulation body.", parameterName);
                }

                for (int existingIndex = 0; existingIndex < i; existingIndex++)
                {
                    if (bodies[existingIndex] == body)
                    {
                        throw new ArgumentException(
                            $"Robot contains duplicate articulation body identities for joints '{names[existingIndex]}' and '{name}'.",
                            parameterName);
                    }
                }

                names[i] = name;
                bodies[i] = body;
                jointTypes[i] = body.jointType;
            }

            return new Metadata(provider, names, bodies, jointTypes, indicesByName);
        }

        internal static bool MetadataMatches(Metadata binding, Metadata metadata)
        {
            if (binding == null
                || metadata == null
                || !ReferenceEquals(binding.Provider, metadata.Provider)
                || binding.Count != metadata.Count)
            {
                return false;
            }

            for (int i = 0; i < binding.Count; i++)
            {
                if (!string.Equals(binding.Names[i], metadata.Names[i], StringComparison.Ordinal)) { return false; }
                if (binding.Bodies[i] == null
                    || metadata.Bodies[i] == null
                    || !ReferenceEquals(binding.Bodies[i], metadata.Bodies[i]))
                {
                    return false;
                }

                if (binding.JointTypes[i] != metadata.JointTypes[i]) { return false; }
            }

            return true;
        }

        internal static NamedVector CaptureNamedVector(
            Metadata metadata,
            string[] names,
            double[] values,
            bool requireEveryRobotJoint,
            bool allowExtraNames,
            string namesParameterName,
            string valuesParameterName,
            string valueName)
        {
            if (names == null) { throw new ArgumentNullException(namesParameterName); }
            if (values == null) { throw new ArgumentNullException(valuesParameterName); }
            if (names.Length != values.Length)
            {
                throw new ArgumentException(
                    $"Joint name and {valueName} vectors must have the same length.",
                    valuesParameterName);
            }

            var sourceNames = new HashSet<string>(StringComparer.Ordinal);
            var representedRobotJoints = new bool[metadata.Count];
            var robotIndices = new int[names.Length];
            var detachedValues = new double[values.Length];
            int representedCount = 0;
            for (int i = 0; i < names.Length; i++)
            {
                string name = names[i];
                if (string.IsNullOrEmpty(name))
                {
                    throw new ArgumentException("Joint names cannot contain null or empty entries.", namesParameterName);
                }

                if (!sourceNames.Add(name))
                {
                    throw new ArgumentException($"Joint vector contains duplicate name '{name}'.", namesParameterName);
                }

                double value = values[i];
                if (!IsFinite(value))
                {
                    throw new ArgumentException($"Joint {valueName} for '{name}' must be finite.", valuesParameterName);
                }

                if (!metadata.IndicesByName.TryGetValue(name, out int robotIndex))
                {
                    if (!allowExtraNames)
                    {
                        throw new ArgumentException($"Robot does not contain joint '{name}'.", namesParameterName);
                    }

                    robotIndices[i] = -1;
                    detachedValues[i] = value;
                    continue;
                }

                robotIndices[i] = robotIndex;
                detachedValues[i] = value;
                representedRobotJoints[robotIndex] = true;
                representedCount++;
            }

            if (requireEveryRobotJoint && representedCount != metadata.Count)
            {
                for (int i = 0; i < representedRobotJoints.Length; i++)
                {
                    if (!representedRobotJoints[i])
                    {
                        throw new ArgumentException(
                            $"Joint vector is missing required joint '{metadata.Names[i]}'.",
                            namesParameterName);
                    }
                }
            }

            return new NamedVector(robotIndices, detachedValues, representedCount == metadata.Count);
        }

        internal static float ToFiniteFloat(double value, string parameterName, string jointName)
        {
            float result = (float)value;
            if (!IsFinite(result))
            {
                throw new ArgumentException($"Joint target for '{jointName}' cannot be represented as a finite float.", parameterName);
            }

            return result;
        }

        internal static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        internal static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
