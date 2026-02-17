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

/*!\file manipulation_pipeline/planning_step.h
 * \brief Base interface for planning tasks defined through actions
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-08
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_PLANNING_STEP_H_INCLUDED
#define MANIPULATION_PIPELINE_PLANNING_STEP_H_INCLUDED

#include "manipulation_pipeline/goal_handle.h"
#include "manipulation_pipeline/robot_model.h"
#include "manipulation_pipeline/visualization_interface.h"

#include <Eigen/Geometry>
#include <functional>
#include <manipulation_pipeline_interfaces/msg/cartesian_limits.hpp>
#include <manipulation_pipeline_interfaces/msg/linear_motion.hpp>
#include <manipulation_pipeline_interfaces/msg/motion_parameters.hpp>
#include <manipulation_pipeline_interfaces/msg/tool_command.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <string>
#include <tf2_ros/buffer_interface.h>
#include <vector>

namespace moveit::core {
class RobotState;
}

namespace manipulation_pipeline {

struct PlanningContext;
class Planner;
class Action;
class ActionSequence;

moveit_cpp::PlanningComponent::PlanRequestParameters applyMotionParameters(
  const manipulation_pipeline_interfaces::msg::MotionParameters& motion_parameters,
  const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params);
manipulation_pipeline_interfaces::msg::CartesianLimits applyCartesianLimits(
  const manipulation_pipeline_interfaces::msg::CartesianLimits& motion_parameters,
  const manipulation_pipeline_interfaces::msg::CartesianLimits& default_limits);

/*! \brief Base class for planning steps
 *
 * For each provided planning action, one child class should be defined.
 */
class PlanningStep
{
public:
  PlanningStep(std::string name, std::shared_ptr<GoalHandle> goal_handle, rclcpp::Logger log);

  [[nodiscard]] const std::string& name() const;
  [[nodiscard]] const std::shared_ptr<GoalHandle>& goalHandle() const;

  [[nodiscard]] virtual moveit_cpp::PlanningComponent::PlanRequestParameters applyRequestParams(
    const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params) const;
  [[nodiscard]] virtual manipulation_pipeline_interfaces::msg::CartesianLimits applyRequestLimits(
    const manipulation_pipeline_interfaces::msg::CartesianLimits& cartesian_limits) const;

  virtual std::shared_ptr<ActionSequence>
  plan(const RobotModel& robot_model,
       const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
       const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
       PlanningContext& context) const = 0;

protected:
  std::shared_ptr<Action>
  createToolAction(const GroupInterface& group_interface,
                   const moveit::core::RobotModelConstPtr& robot_model,
                   const Planner& planner,
                   moveit::core::RobotState& current_state,
                   const manipulation_pipeline_interfaces::msg::ToolCommand& cmd) const;
  std::vector<Eigen::Isometry3d>
  convertLinearMotion(const manipulation_pipeline_interfaces::msg::LinearMotion& msg,
                      const Eigen::Isometry3d& offset = Eigen::Isometry3d::Identity()) const;

  rclcpp::Logger m_log;

private:
  std::string m_name;

  std::shared_ptr<GoalHandle> m_goal_handle;
};

/*! \brief Action-Specific base class for action planning steps
 *
 * \tparam ActionT ROS action type
 */
template <typename ActionT>
class ActionPlanningStep : public PlanningStep
{
public:
  ActionPlanningStep(const std::shared_ptr<ActionGoalHandle<ActionT>>& goal_handle,
                     std::string name,
                     rclcpp::Logger log);

protected:
  const typename ActionT::Goal* m_goal;
};


} // namespace manipulation_pipeline


namespace manipulation_pipeline {

template <typename ActionT>
ActionPlanningStep<ActionT>::ActionPlanningStep(
  const std::shared_ptr<ActionGoalHandle<ActionT>>& goal_handle,
  std::string name,
  rclcpp::Logger log)
  : PlanningStep{std::move(name), goal_handle, std::move(log)}
  , m_goal{goal_handle->handle().get_goal().get()}
{
}

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_PLANNING_STEP_H_INCLUDED
