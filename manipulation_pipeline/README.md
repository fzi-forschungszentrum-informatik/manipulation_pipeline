Manipulation Pipeline
---------------------

This package provides simple ROS interfaces for motion planning and manipulation based on [MoveIt 2](https://moveit.picknik.ai).

Interfaces
----------

The manipulation pipeline node provides the following interfaces:

- Topic `state` ([State](../manipulation_pipeline_interfaces/msg/State.msg)): Current state of the manipulation pipeline. In particular, the generation count can be used to wait for the pipeline to restart after a robot description change.
- Action `actuate_tool` ([ActuateTool](../manipulation_pipeline_interfaces/action/ActuateTool.action)): Actuate a tool identified by its end-effector name. This can be used to verify tool functions before performing manipulation tasks.
- Action `move_to_named_pose` ([MoveToNamedPose](../manipulation_pipeline_interfaces/action/MoveToNamedPose.action)): Move to a named pose as defined in the SRDF.
- Action `move_to_pose` ([MoveToPose](../manipulation_pipeline_interfaces/action/MoveToPose.action)): Move a link to a cartesian pose.
- Action `execute_path` ([ExecutePath](../manipulation_pipeline_interfaces/action/ExecutePath.action)): Use the [pilz_industrial_motion_planner](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html) to move a link linearly along a cartesian path.
- Action `grasp` ([Grasp](../manipulation_pipeline_interfaces/action/Grasp.action)): Move a link to an object, do a linear cartesian approach, actuate a tool and retract.
- Action `place` ([Place](../manipulation_pipeline_interfaces/action/Place.action)): Place a previously grapsed object at a specific pose by moving there, doing a cartesian approach, actuating a tool and retracting.
- Service `spawn_object` ([SpawnObject](../manipulation_pipeline_interfaces/srv/SpawnObject.srv)): Spawn a collision object in the MoveIt planning scene.

The manipulation pipeline constantly listens to the `robot_description` and `robot_description_semantic` topics.
As soon as new descriptions are published, it restarts.
This enables dynamic adjustments to the URDF and SRDF, e.g. to implement tool changer support.

Configuration
-------------

This package completely relies on a functioning MoveIt 2 setup and requires no further configuration.
Especially relevant for its configuration are:
- SRDF groups: All motion actions are specified based on joint groups from the SRDF.
    - Chains: Actions that rely on inverse kinematics (e.g. `move_to_pose`) can only be called with groups that are defined as a single `<chain />` element.
    - End effectors: All actions that actuate tools (e.g. `grasp`) are specified based on end effector tags.
- MoveItCpp config (`moveit_cpp.yaml`): The values specified here are used as default if no motion parameters are defined in an action.
- Pilz config (`pilz_cartesian_limits.yaml`): This package uses the `pilz_industrial_motion_planner` to execute cartesian motions. The values specified here define the default execution speed.
