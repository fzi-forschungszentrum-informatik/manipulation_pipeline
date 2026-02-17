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
#include "manipulation_pipeline/planning_step.h"

#include "manipulation_pipeline/actions/dwell.h"
#include "manipulation_pipeline/actions/execute_trajectory.h"
#include "manipulation_pipeline/planner.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

moveit_cpp::PlanningComponent::PlanRequestParameters applyMotionParameters(
  const manipulation_pipeline_interfaces::msg::MotionParameters& motion_parameters,
  const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params)
{
  auto result = default_params;

  const auto goal_max_vel = motion_parameters.max_velocity_scaling;
  if (goal_max_vel != -1.0)
  {
    if ((goal_max_vel < 0.0) || (goal_max_vel > 1.0))
    {
      throw std::runtime_error{
        "Invalid motion parameter: Max velocity can either be -1.0 or within [0.0, 1.0]"};
    }

    result.max_velocity_scaling_factor = goal_max_vel;
  }

  const auto goal_max_acc = motion_parameters.max_acceleration_scaling;
  if (goal_max_acc != -1.0)
  {
    if ((goal_max_acc < 0.0) || (goal_max_acc > 1.0))
    {
      throw std::runtime_error{
        "Invalid motion parameter: Max acceleration can either be -1.0 or within [0.0, 1.0]"};
    }

    result.max_acceleration_scaling_factor = goal_max_acc;
  }

  return result;
}

manipulation_pipeline_interfaces::msg::CartesianLimits applyCartesianLimits(
  const manipulation_pipeline_interfaces::msg::CartesianLimits& motion_parameters,
  const manipulation_pipeline_interfaces::msg::CartesianLimits& default_limits)
{
  manipulation_pipeline_interfaces::msg::CartesianLimits result = motion_parameters;
  const auto check_default                                      = [](double& v, double default_v) {
    if (v == 0.0)
    {
      v = default_v;
    }
  };

  check_default(result.max_trans_vel, default_limits.max_trans_vel);
  check_default(result.max_trans_acc, default_limits.max_trans_acc);
  check_default(result.max_trans_dec, default_limits.max_trans_dec);
  check_default(result.max_rot_vel, default_limits.max_rot_vel);

  return result;
}

PlanningStep::PlanningStep(std::string name,
                           std::shared_ptr<GoalHandle> goal_handle,
                           rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_name{std::move(name)}
  , m_goal_handle{std::move(goal_handle)}
{
}

const std::string& PlanningStep::name() const
{
  return m_name;
}

const std::shared_ptr<GoalHandle>& PlanningStep::goalHandle() const
{
  return m_goal_handle;
}

moveit_cpp::PlanningComponent::PlanRequestParameters PlanningStep::applyRequestParams(
  const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params) const
{
  return default_params;
}

manipulation_pipeline_interfaces::msg::CartesianLimits PlanningStep::applyRequestLimits(
  const manipulation_pipeline_interfaces::msg::CartesianLimits& cartesian_limits) const
{
  return cartesian_limits;
}

std::shared_ptr<Action>
PlanningStep::createToolAction(const GroupInterface& group_interface,
                               const moveit::core::RobotModelConstPtr& robot_model,
                               const Planner& planner,
                               moveit::core::RobotState& current_state,
                               const manipulation_pipeline_interfaces::msg::ToolCommand& cmd) const
{
  switch (cmd.command)
  {
    case manipulation_pipeline_interfaces::msg::ToolCommand::COMMAND_JOINT: {
      const auto trajectory =
        std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_interface.group());

      trajectory->addSuffixWayPoint(current_state, 1.0);

      current_state.setVariableValues(cmd.joint_cmd);
      trajectory->addSuffixWayPoint(current_state, 1.0);

      planner.retimeTrajectory(*trajectory);

      return std::make_shared<actions::ExecuteTrajectory>(
        trajectory, cmd.controller, std::vector<const moveit::core::LinkModel*>{}, "ToolCommand");
    }

    case manipulation_pipeline_interfaces::msg::ToolCommand::COMMAND_DWELL:
      return std::make_shared<actions::Dwell>(std::chrono::duration<double>{cmd.dwell_duration},
                                              fmt::format("Dwell({:.02f}s)", cmd.dwell_duration));

    case manipulation_pipeline_interfaces::msg::ToolCommand::COMMAND_NONE:
      return std::shared_ptr<Action>{};

    default:
      throw std::runtime_error{fmt::format("Invalid tool command '{}'", cmd.command)};
  }
}

std::vector<Eigen::Isometry3d>
PlanningStep::convertLinearMotion(const manipulation_pipeline_interfaces::msg::LinearMotion& msg,
                                  const Eigen::Isometry3d& offset) const
{
  std::vector<Eigen::Isometry3d> waypoints;

  switch (msg.motion_type)
  {
    case manipulation_pipeline_interfaces::msg::LinearMotion::MOTION_TYPE_LINEAR: {
      const auto dist = (msg.distance == 0.0) ? 0.1 : msg.distance;
      waypoints.push_back(offset * Eigen::Isometry3d{Eigen::Translation3d{0, 0, -dist}});
      break;
    }

    case manipulation_pipeline_interfaces::msg::LinearMotion::MOTION_TYPE_PATH: {
      Eigen::Isometry3d buf;
      waypoints.reserve(msg.waypoints.size());
      std::transform(msg.waypoints.begin(),
                     msg.waypoints.end(),
                     std::back_inserter(waypoints),
                     [&](const auto& msg) {
                       tf2::convert(msg, buf);
                       return offset * buf;
                     });
      break;
    }

    default:
      throw std::runtime_error{fmt::format("Unknon linear motion type {}", msg.motion_type)};
  }

  return waypoints;
}

} // namespace manipulation_pipeline
