Manipulation Pipeline
=====================

This package provides simple ROS interfaces for motion planning and manipulation based on [MoveIt 2](https://moveit.picknik.ai).

---

This package is still work in progress and neither feature complete nor tested in its entirety.

---

## Build Status

<table width="100%">
    <tr>
        <th>
            <a href="https://docs.ros.org/en/jazzy">Jazzy</a>
        </th>
        <th>
            <a href="https://docs.ros.org/en/rolling">Rolling</a>
        </th>
    </tr>
    <tr>
        <td>
            <a href="https://github.com/fzi-forschungszentrum-informatik/manipulation_pipeline/actions/workflows/jazzy_main.yml">
              <img src="https://github.com/fzi-forschungszentrum-informatik/manipulation_pipeline/actions/workflows/jazzy_main.yml/badge.svg?branch=main"alt="Jazzy Main"/>
            </a> <br />
            <a href="https://github.com/fzi-forschungszentrum-informatik/manipulation_pipeline/actions/workflows/jazzy_testing.yml">
              <img src="https://github.com/fzi-forschungszentrum-informatik/manipulation_pipeline/actions/workflows/jazzy_testing.yml/badge.svg?branch=main"alt="Jazzy Testing"/>
            </a>
        </td>
        <td>
            <a href="https://github.com/fzi-forschungszentrum-informatik/manipulation_pipeline/actions/workflows/rolling_main.yml">
              <img src="https://github.com/fzi-forschungszentrum-informatik/manipulation_pipeline/actions/workflows/rolling_main.yml/badge.svg?branch=main"alt="Rolling Main"/>
            </a> <br />
            <a href="https://github.com/fzi-forschungszentrum-informatik/manipulation_pipeline/actions/workflows/rolling_testing.yml">
              <img src="https://github.com/fzi-forschungszentrum-informatik/manipulation_pipeline/actions/workflows/rolling_testing.yml/badge.svg?branch=main"alt="Rolling Testing"/>
            </a>
        </td>
    </tr>
</table>

Features
--------

On top of MoveIt, this package provides:
- Simplified planning interfaces for point-to-point and cartesian planning.
- Pipelined planning and execution, enabling multi-step planning without pauses.
- Support for URDF/SRDF changes during runtime, enabling tool changer support.

It tries to require as little as possible configuration in addition to a working `_moveit_config` package and follow normal MoveIt concepts while providing sane defaults.
As an example, for the `MoveToPose` action this means:
- The `joint_group` field specifies the SRDF group that should be moved to a target. While this needs to be a chain (to enable kinematic calculations), if you only have one non-endeffector group this can be left empty.
- The `tip` field specifies which link needs to be moved to the pose, but defaults to the end of the `joint_group` tip.
- `motion_parameters` can specify the time parametrization, but if left empty the normal `moveit_cpp` config from the `_moveit_config` package is used.
- `controller` can specify a specific controller to use, but normal MoveIt controller default rules are used if this is not specified.

As a result, many setups might allow you to only specify a target pose without worrying about the rest.

Packages in the Repository
--------------------------

- `manipulation_pipeline`: `MoveItCpp` based motion planning and manipulation node.
- `manipulation_pipeline_interfaces`: Interfaces for interacting with the manipulation pipeline.

Acknowledgements
----------------

Development of this driver was in parts supported by the project "GANResilRob - Generative Adversarial Networks and Semantics for Resilient, Flexible Production Robots", funded by the German Federal Ministry for Economic Affairs and Energy under grant agreement 01MJ22003A.
