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
import logging
import time

import rclpy
import rclpy.node
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from manipulation_pipeline_interfaces.action import MoveToNamedPose, MoveToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration


def _wait_for_service(node, service_type, service_name, timeout_sec):
    client = node.create_client(service_type, service_name)

    logging.info(f"Waiting for service {service_name} (timeout={timeout_sec:.1f}s)...")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(
            f"Service {service_name} not available after {timeout_sec:.1f} seconds"
        )
    logging.info(f"Service {service_name} ready")

    return client


def _wait_for_action(node, action_type, action_name, timeout_sec):
    client = ActionClient(node, action_type, action_name)

    logging.info(
        f"Waiting for action server {action_name} (timeout={timeout_sec:.1f}s)..."
    )
    if not client.wait_for_server(timeout_sec=timeout_sec):
        raise RuntimeError(
            f"Action server {action_name} not available after {timeout_sec:.1f} seconds"
        )
    logging.info(f"Action server {action_name} ready")

    return client


class MapiClient:
    _ACTIONS = {
        "move_to_named_pose": MoveToNamedPose,
        "move_to_pose": MoveToPose,
    }

    def __init__(self, node: rclpy.node.Node, ns: str = "manipulation_pipeline"):
        self._node = node

        self._list_controllers_client = _wait_for_service(
            node,
            ListControllers,
            "/controller_manager/list_controllers",
            timeout_sec=30.0,
        )
        self._wait_for_controller("joint_trajectory_controller", timeout_sec=30.0)

        for action_name, action_type in self._ACTIONS.items():
            client = _wait_for_action(
                node, action_type, f"{ns}/{action_name}", timeout_sec=20.0
            )

            def make_method(c, t, expected_result):
                def method(**kwargs):
                    return self._call_action(
                        c, t, expected_result=expected_result, **kwargs
                    )

                return method

            setattr(
                self,
                f"{action_name}",
                make_method(client, action_type, expected_result="success"),
            )
            setattr(
                self,
                f"{action_name}_expect_abort",
                make_method(client, action_type, expected_result="abort"),
            )

    def _wait_for_controller(self, controller_name, timeout_sec):
        deadline = self._node.get_clock().now() + Duration(seconds=timeout_sec)
        logging.info(
            f"Waiting for controller {controller_name} to be active (timeout={timeout_sec:.1f}s)..."
        )

        while self._node.get_clock().now() < deadline:
            future = self._list_controllers_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            if future.done():
                for controller in future.result().controller:
                    if (
                        controller.name == controller_name
                        and controller.state == "active"
                    ):
                        logging.info(f"Controller {controller_name} is active")
                        return
            rclpy.spin_once(self._node, timeout_sec=1.0)
            time.sleep(1.0)

        raise RuntimeError(
            f"Controller {controller_name} not active after {timeout_sec:.1f} seconds"
        )

    def _call_action(
        self,
        client,
        action_type,
        expected_result="success",
        **kwargs,
    ):
        goal = action_type.Goal(**kwargs)
        logging.info(f"Sending goal to {client._action_name}: {goal}")

        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, goal_future)
        goal_handle = goal_future.result()
        assert goal_handle.accepted
        logging.info(f"Goal accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=10.0)
        assert result_future.done()
        result = result_future.result()
        logging.info(f"Received result: {result}")

        if expected_result == "success":
            assert result.status == GoalStatus.STATUS_SUCCEEDED
            assert result.result.success
        if expected_result == "abort":
            assert result.status == GoalStatus.STATUS_ABORTED
            assert not result.result.success

        return result
