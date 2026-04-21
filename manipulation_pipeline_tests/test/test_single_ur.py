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
from math import sqrt

import launch_pytest
import pytest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from std_msgs.msg import Header


@launch_pytest.fixture(scope="module")
def launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("manipulation_pipeline_tests"),
                                "launch",
                                "single_ur.launch.py",
                            ]
                        )
                    ]
                )
            ),
            ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_move_to_named_pose(mapi_client):
    mapi_client.move_to_named_pose(pose_name="home")
    mapi_client.move_to_named_pose_expect_abort(pose_name="non_existent_pose")
    mapi_client.move_to_named_pose(pose_name="test_1", joint_group="ur")
    mapi_client.move_to_named_pose_expect_abort(
        pose_name="home", joint_group="none_existent_group"
    )
    mapi_client.move_to_named_pose(pose_name="test_2")


@pytest.mark.launch(fixture=launch_description)
def test_move_to_pose(mapi_client):
    mapi_client.move_to_pose(
        pose=PoseStamped(
            header=Header(frame_id="world"),
            pose=Pose(
                position=Point(y=0.7, z=0.5),
                orientation=Quaternion(x=-sqrt(2) / 2, w=sqrt(2) / 2),
            ),
        )
    )
    mapi_client.move_to_pose(
        pose=PoseStamped(
            header=Header(frame_id="ur_base_link"),
            pose=Pose(
                position=Point(x=0.6, z=0.5),
                orientation=Quaternion(x=-1.0, w=0.0),
            ),
        )
    )
