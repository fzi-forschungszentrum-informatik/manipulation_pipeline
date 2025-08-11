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
 * \date    2025-04-29
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/planning_steps/actuate_tool.h"

#include "manipulation_pipeline/action.h"
#include "manipulation_pipeline/actions/execute_trajectory.h"
#include "manipulation_pipeline/planning_context.h"

namespace manipulation_pipeline {

ActuateTool::ActuateTool(const std::shared_ptr<Handle>& handle, rclcpp::Logger log)
  : ActionPlanningStep<Action>{
      handle,
      fmt::format("ActuateTool({})", handle->handle().get_goal()->end_effector),
      std::move(log)}
{
}

moveit_cpp::PlanningComponent::PlanRequestParameters ActuateTool::applyRequestParams(
  const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params) const
{
  return default_params;
}

std::shared_ptr<ActionSequence>
ActuateTool::plan(const RobotModel& robot_model,
                  const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
                  const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
                  PlanningContext& context) const
{
  const auto& group_interface = robot_model.findEndEffector(m_goal->end_effector);

  const auto action_sequence = std::make_shared<ActionSequence>(name(), goalHandle());

  auto robot_state = context.planning_scene->getCurrentState();

  const auto tool_action = createToolAction(
    group_interface, context.planning_scene->getRobotModel(), robot_state, m_goal->command);
  if (tool_action)
  {
    action_sequence->add(tool_action);
  }

  context.planning_scene->setCurrentState(robot_state);

  return action_sequence;
}

} // namespace manipulation_pipeline
