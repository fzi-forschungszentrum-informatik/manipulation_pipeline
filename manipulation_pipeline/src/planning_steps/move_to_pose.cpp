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
 * \date    2025-04-10
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/planning_steps/move_to_pose.h"

#include "manipulation_pipeline/action.h"
#include "manipulation_pipeline/actions/execute_trajectory.h"
#include "manipulation_pipeline/planner.h"
#include "manipulation_pipeline/planning_context.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

MoveToPose::MoveToPose(const std::shared_ptr<Handle>& handle, rclcpp::Logger log)
  : ActionPlanningStep<Action>{handle, "MoveToPose()", std::move(log)}
{
}

moveit_cpp::PlanningComponent::PlanRequestParameters MoveToPose::applyRequestParams(
  const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params) const
{
  return applyMotionParameters(m_goal->motion_parameters, default_params);
}

std::shared_ptr<ActionSequence>
MoveToPose::plan(const RobotModel& robot_model,
                 const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
                 const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
                 PlanningContext& context) const
{
  auto& group_interface = robot_model.findChain(m_goal->joint_group);
  Planner planner{group_interface, context, params, limits, m_log};

  // Transform target pose to model frame
  if (!context.planning_scene->knowsFrameTransform(m_goal->pose.header.frame_id))
  {
    throw std::runtime_error{
      fmt::format("Unknown collision object target pose frame '{}'", m_goal->pose.header.frame_id)};
  }
  const auto target_pose_frame_transform =
    context.planning_scene->getFrameTransform(m_goal->pose.header.frame_id);

  Eigen::Isometry3d target_pose_local;
  tf2::convert(m_goal->pose.pose, target_pose_local);

  const auto target_pose = target_pose_frame_transform * target_pose_local;

  // Plan trajectory
  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = context.planning_scene->getPlanningFrame();
  tf2::convert(target_pose, target_pose_msg.pose);

  const auto initial_state = context.planning_scene->getCurrentState();
  const auto trajectory    = planner.plan(initial_state,
                                       target_pose_msg,
                                       group_interface.resolveTip(m_goal->tip),
                                       context.planning_scene);
  if (!trajectory)
  {
    throw std::runtime_error{fmt::format("Could not plan to pose: {}", planner.lastErrorMsg())};
  }

  // Bookkeeping
  context.planning_scene->setCurrentState(trajectory->getLastWayPoint());

  return std::make_shared<ActionSequence>(
    name(),
    std::make_shared<actions::ExecuteTrajectory>(
      trajectory, m_goal->controller, group_interface.tipLinks(), "PTP"),
    goalHandle());
}

} // namespace manipulation_pipeline
