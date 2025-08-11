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
 * \date    2025-04-07
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/planning_pipeline.h"

#include "manipulation_pipeline/action.h"
#include "manipulation_pipeline/planning_context.h"
#include "manipulation_pipeline/planning_step.h"

#include <fmt/format.h>
#include <manipulation_pipeline_interfaces/msg/progress.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>

namespace manipulation_pipeline {

PlanningPipeline::PlanningPipeline(moveit_cpp::MoveItCppPtr moveit_cpp,
                                   std::shared_ptr<RobotModel> robot_model,
                                   std::shared_ptr<ExecutionPipeline> execution_pipeline,
                                   std::shared_ptr<VisualizationInterface> visualization_interface,
                                   const rclcpp::Node::SharedPtr& node,
                                   rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_node{node}
  , m_default_cartesian_limits{loadCartesianLimits(*node)}
  , m_moveit_cpp{std::move(moveit_cpp)}
  , m_robot_model{std::move(robot_model)}
  , m_execution_pipeline{std::move(execution_pipeline)}
  , m_visualization_interface{std::move(visualization_interface)}
  , m_worker{std::bind(&PlanningPipeline::run, this, std::placeholders::_1, std::placeholders::_2)}
{
  m_default_params.load(node);

  const auto default_param_str = printParameters(m_default_params, m_default_cartesian_limits);
  RCLCPP_INFO(m_log, "Using default parameters: %s", default_param_str.c_str());
}

void PlanningPipeline::addPlanningStep(const std::shared_ptr<PlanningStep> step)
{
  step->goalHandle()->feedback(
    manipulation_pipeline_interfaces::msg::Progress::PROGRESS_PLANNING_QUEUED,
    fmt::format("Planning step '{}' queued for planning", step->name()));
  m_worker.push(step);
}

manipulation_pipeline_interfaces::msg::CartesianLimits
PlanningPipeline::loadCartesianLimits(const rclcpp::Node& node) const
{
  const std::string param_prefix = "robot_description_planning.cartesian_limits.";

  bool params_found = true;

  manipulation_pipeline_interfaces::msg::CartesianLimits limits;
  params_found =
    params_found && node.get_parameter(param_prefix + "max_trans_vel", limits.max_trans_vel);
  params_found =
    params_found && node.get_parameter(param_prefix + "max_trans_acc", limits.max_trans_acc);
  params_found =
    params_found && node.get_parameter(param_prefix + "max_trans_dec", limits.max_trans_dec);
  params_found =
    params_found && node.get_parameter(param_prefix + "max_rot_vel", limits.max_rot_vel);

  if (!params_found)
  {
    throw std::runtime_error{"Missing cartesian limit parameter"};
  }

  return limits;
}

void PlanningPipeline::run(const std::shared_ptr<PlanningStep>& step, std::stop_token stop_token)
{
  try
  {
    RCLCPP_INFO(m_log, "Planning step '%s'", step->name().c_str());
    step->goalHandle()->feedback(manipulation_pipeline_interfaces::msg::Progress::PROGRESS_PLANNING,
                                 fmt::format("Planning step '{}'", step->name()));

    // Choose correct planning scene
    const auto planning_scene = [&] {
      if (m_last_goal && m_last_planning_scene)
      {
        if (!m_last_goal->isDone())
        {
          RCLCPP_INFO(m_log, "Reusing previous planning scene");
          return m_last_planning_scene;
        }
      }

      RCLCPP_INFO(m_log, "Cloning current planning scene");
      auto planning_scene_monitor = m_moveit_cpp->getPlanningSceneMonitorNonConst();
      planning_scene_monitor->updateFrameTransforms();

      planning_scene_monitor::LockedPlanningSceneRO planning_scene{planning_scene_monitor};
      return planning_scene::PlanningScene::clone(planning_scene);
    }();

    planning_scene->getCurrentStateNonConst().update();

    // Get relevant planning parameters
    const auto planning_params     = step->applyRequestParams(m_default_params);
    const auto cartesian_limits    = step->applyRequestLimits(m_default_cartesian_limits);
    const auto planning_params_str = printParameters(planning_params, cartesian_limits);
    RCLCPP_INFO(m_log, "Planning parameters: %s", planning_params_str.c_str());

    // Clear planning visualization
    m_visualization_interface->plan_interface->clear();

    // Plan trajectory
    PlanningContext context{m_node,
                            planning_scene,
                            m_moveit_cpp->getPlanningPipelines(),
                            m_visualization_interface->plan_interface.get()};

    const auto actions = step->plan(*m_robot_model, planning_params, cartesian_limits, context);
    actions->visualize(m_visualization_interface->trajectory_interface);

    // Bookkeeping for pipelined execution
    m_last_goal           = step->goalHandle();
    m_last_planning_scene = planning_scene;

    // Execute trajectory
    m_execution_pipeline->execute(actions);
  }
  catch (const std::runtime_error& ex)
  {
    const auto error_msg = fmt::format("Planning error: {}", ex.what());
    RCLCPP_ERROR(m_log, "%s", error_msg.c_str());
    step->goalHandle()->failure(error_msg);
  }
}

std::string PlanningPipeline::printParameters(
  const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
  const manipulation_pipeline_interfaces::msg::CartesianLimits& cartesian_limits) const
{
  return fmt::format("{}(planner={},#attempts={},planning_time={:.02f}s,max_vel_factor={:.02f},max_"
                     "acc_factor={:.02f}),"
                     "cartesian(max_trans_vel={:.02f},max_trans_acc={:.02f},max_trans_dec={:.02f},"
                     "max_rot_vel={:.02f})",
                     params.planning_pipeline,
                     params.planner_id,
                     params.planning_attempts,
                     params.planning_time,
                     params.max_velocity_scaling_factor,
                     params.max_acceleration_scaling_factor,
                     cartesian_limits.max_trans_vel,
                     cartesian_limits.max_trans_acc,
                     cartesian_limits.max_trans_dec,
                     cartesian_limits.max_rot_vel);
}

} // namespace manipulation_pipeline
