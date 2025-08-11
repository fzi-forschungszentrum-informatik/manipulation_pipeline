// Copyright 2025 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!\file
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-08
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/planning_interface.h"

#include "manipulation_pipeline/planning_steps/actuate_tool.h"
#include "manipulation_pipeline/planning_steps/execute_path.h"
#include "manipulation_pipeline/planning_steps/grasp.h"
#include "manipulation_pipeline/planning_steps/move_to_named_pose.h"
#include "manipulation_pipeline/planning_steps/move_to_pose.h"
#include "manipulation_pipeline/planning_steps/place.h"

namespace manipulation_pipeline {

PlanningInterface::PlanningInterface(std::shared_ptr<PlanningPipeline> planning_pipeline,
                                     const rclcpp::Node::SharedPtr& node,
                                     rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_planning_pipeline{std::move(planning_pipeline)}
  , m_goal_cnt{0}
  , m_move_to_named_pose{rclcpp_action::create_server<MoveToNamedPoseAction>(
      node,
      "~/move_to_named_pose",
      [this](const auto& uuid, auto goal) {
        return actionGoalCb<MoveToNamedPoseAction>(uuid, goal);
      },
      [this](auto goal_handle) { return actionCancelCb(goal_handle); },
      [this](auto goal_handle) {
        actionAcceptCb<MoveToNamedPoseAction, MoveToNamedPose>(goal_handle);
      })}
  , m_move_to_pose{rclcpp_action::create_server<MoveToPoseAction>(
      node,
      "~/move_to_pose",
      [this](const auto& uuid, auto goal) { return actionGoalCb<MoveToPoseAction>(uuid, goal); },
      [this](auto goal_handle) { return actionCancelCb(goal_handle); },
      [this](auto goal_handle) { actionAcceptCb<MoveToPoseAction, MoveToPose>(goal_handle); })}
  , m_grasp{rclcpp_action::create_server<GraspAction>(
      node,
      "~/grasp",
      [this](const auto& uuid, auto goal) { return actionGoalCb<GraspAction>(uuid, goal); },
      [this](auto goal_handle) { return actionCancelCb(goal_handle); },
      [this](auto goal_handle) { actionAcceptCb<GraspAction, Grasp>(goal_handle); })}
  , m_place{rclcpp_action::create_server<PlaceAction>(
      node,
      "~/place",
      [this](const auto& uuid, auto goal) { return actionGoalCb<PlaceAction>(uuid, goal); },
      [this](auto goal_handle) { return actionCancelCb(goal_handle); },
      [this](auto goal_handle) { actionAcceptCb<PlaceAction, Place>(goal_handle); })}
  , m_actuate_tool{rclcpp_action::create_server<ActuateToolAction>(
      node,
      "~/actuate_tool",
      [this](const auto& uuid, auto goal) { return actionGoalCb<ActuateToolAction>(uuid, goal); },
      [this](auto goal_handle) { return actionCancelCb(goal_handle); },
      [this](auto goal_handle) { actionAcceptCb<ActuateToolAction, ActuateTool>(goal_handle); })}
  , m_execute_path{rclcpp_action::create_server<ExecutePathAction>(
      node,
      "~/execute_path",
      [this](const auto& uuid, auto goal) { return actionGoalCb<ExecutePathAction>(uuid, goal); },
      [this](auto goal_handle) { return actionCancelCb(goal_handle); },
      [this](auto goal_handle) { actionAcceptCb<ExecutePathAction, ExecutePath>(goal_handle); })}
{
}

} // namespace manipulation_pipeline
