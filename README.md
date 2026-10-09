# Unity x ROS Integration Package

Collection of scripts for integrating the digital twins of ROS-Industrial robots based on ROS TCP Connector and URDF Importer packages from Unity.

More detailed documentation [can be found here](https://jgroxz.notion.site/Unity-x-ROS-Wiki-c2c7bf7a271946d694bfa30a5aca4822).

## Joint-state mirroring

Joint-state mirroring is split into three components:

1. `RobotJointStatesListener` subscribes to `sensor_msgs/JointState`, validates and maps values by exact joint name, and exposes telemetry.
2. `RobotJointStateMirror` forwards complete validated snapshots to a Unity motion controller.
3. `UrdfRobotMotionController` owns target admission and applies the targets to the robot's articulation hierarchy.

To mirror a ROS robot in a scene, configure the listener's ROS connection, Unity robot provider, and topic; then add a `RobotJointStateMirror` and assign both the listener and the corresponding `UrdfRobotMotionController`.

### Upgrading existing scenes

`RobotJointStatesListener` no longer contains a motion-controller field. Existing scenes and prefabs that used listener-driven mirroring must add and wire a `RobotJointStateMirror` explicitly, then be saved to remove the obsolete serialized field. `RosIndustrialRobotJointStatesListener` remains available as an obsolete source-compatible alias, but it does not restore the old direct listener-to-controller wiring.
