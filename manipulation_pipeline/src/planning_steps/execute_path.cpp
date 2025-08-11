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
 * \date    2025-04-30
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/planning_steps/execute_path.h"

#include "manipulation_pipeline/action.h"
#include "manipulation_pipeline/actions/execute_trajectory.h"
#include "manipulation_pipeline/planner.h"
#include "manipulation_pipeline/planning_context.h"

#include <fmt/format.h>
#include <moveit/planning_scene/planning_scene.hpp>

namespace manipulation_pipeline {

ExecutePath::ExecutePath(const std::shared_ptr<Handle>& handle, rclcpp::Logger log)
  : ActionPlanningStep<Action>{
      handle,
      fmt::format("ExecutePath({})", handle->handle().get_goal()->joint_group),
      std::move(log)}
{
}

manipulation_pipeline_interfaces::msg::CartesianLimits ExecutePath::applyRequestLimits(
  const manipulation_pipeline_interfaces::msg::CartesianLimits& cartesian_limits) const
{
  return applyCartesianLimits(m_goal->motion_limits, cartesian_limits);
}

std::shared_ptr<ActionSequence>
ExecutePath::plan(const RobotModel& robot_model,
                  const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
                  const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
                  PlanningContext& context) const
{
  auto& group_interface = robot_model.findChain(m_goal->joint_group);
  Planner planner{group_interface, context, params, limits, m_log};

  const auto initial_state = context.planning_scene->getCurrentState();

  const auto planning_scene = planning_scene::PlanningScene::clone(context.planning_scene);
  auto& planning_scene_acm  = planning_scene->getAllowedCollisionMatrixNonConst();
  for (const auto& disabled_collision : m_goal->disabled_collisions)
  {
    planning_scene_acm.setEntry(disabled_collision.link1, disabled_collision.link2, true);
  }

  // Visualize path
  context.plan_visualizer->addPath(m_goal->path, m_goal->path_frame);
  context.plan_visualizer->publish();

  const auto trajectory = planner.planCartesianSequence(initial_state,
                                                        m_goal->path_frame,
                                                        m_goal->path,
                                                        group_interface.resolveTip(m_goal->tip),
                                                        planning_scene);
  if (!trajectory)
  {
    throw std::runtime_error{
      fmt::format("Could not plan cartesian trajectory: {}", planner.lastErrorMsg())};
  }

  // Bookkeeping
  context.planning_scene->setCurrentState(trajectory->getLastWayPoint());

  trajectory->setWayPointDurationFromPrevious(0, 1.0);

  return std::make_shared<ActionSequence>(
    name(),
    std::make_shared<actions::ExecuteTrajectory>(
      trajectory, m_goal->controller, group_interface.tipLinks(), "CartesianPath"),
    goalHandle());
}

} // namespace manipulation_pipeline
