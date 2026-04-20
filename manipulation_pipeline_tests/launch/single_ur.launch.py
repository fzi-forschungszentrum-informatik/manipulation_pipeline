# Copyright 2026 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = FindPackageShare("manipulation_pipeline_tests")

    launch_description = LaunchDescription()

    # Start driver
    launch_description.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                ParameterFile(
                    PathJoinSubstitution(
                        [pkg, "config", "single_ur", "controllers.yaml"]
                    ),
                    allow_substs=True,
                ),
            ],
            output="screen",
            emulate_tty=True,
        )
    )

    launch_description.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {
                    "robot_description": Command(
                        [
                            FindExecutable(name="xacro"),
                            " ",
                            PathJoinSubstitution([pkg, "urdf", "single_ur.urdf.xacro"]),
                        ]
                    )
                }
            ],
            output="screen",
            emulate_tty=True,
        )
    )

    # Spawn controllers
    launch_description.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "joint_trajectory_controller",
            ],
            output="screen",
            emulate_tty=True,
        )
    )

    # Launch mapi
    moveit_config = (
        MoveItConfigsBuilder("single_ur", package_name="manipulation_pipeline_tests")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor()
        .trajectory_execution(file_path="config/single_ur/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/single_ur/kinematics.yaml")
        .joint_limits(file_path="config/single_ur/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/single_ur/pilz_cartesian_limits.yaml")
        .moveit_cpp(file_path="config/single_ur/moveit_cpp.yaml")
    ).to_moveit_configs()

    launch_description.add_action(
        Node(
            package="manipulation_pipeline",
            executable="manipulation_pipeline",
            name="manipulation_pipeline",
            parameters=[moveit_config.to_dict()],
            output="screen",
            emulate_tty=True,
        )
    )

    return launch_description
