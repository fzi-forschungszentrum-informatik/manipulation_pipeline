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

/*!\file manipulation_pipeline/planning_pipeline.h
 * \brief Pipelined planning pipeline for action planning steps
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-07
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_PLANNING_PIPELINE_H_INCLUDED
#define MANIPULATION_PIPELINE_PLANNING_PIPELINE_H_INCLUDED

#include "execution_pipeline.h"
#include "visualization_interface.h"
#include "worker_thread.h"

#include <manipulation_pipeline/robot_model.h>
#include <manipulation_pipeline_interfaces/msg/cartesian_limits.hpp>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace planning_scene {
class PlanningScene;
}

namespace manipulation_pipeline {

class PlanningStep;
class GoalHandle;

/*! \brief Pipelined planner for action planning steps
 */
class PlanningPipeline
{
public:
  PlanningPipeline(moveit_cpp::MoveItCppPtr moveit_cpp,
                   std::shared_ptr<RobotModel> robot_model,
                   std::shared_ptr<ExecutionPipeline> execution_pipeline,
                   std::shared_ptr<VisualizationInterface> visualization_interface,
                   const rclcpp::Node::SharedPtr& node,
                   rclcpp::Logger log);

  void addPlanningStep(const std::shared_ptr<PlanningStep> step);

private:
  void run(const std::shared_ptr<PlanningStep>& step, std::stop_token stop_token);

  manipulation_pipeline_interfaces::msg::CartesianLimits
  loadCartesianLimits(const rclcpp::Node& node) const;
  std::string printParameters(
    const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
    const manipulation_pipeline_interfaces::msg::CartesianLimits& cartesian_limits) const;

  rclcpp::Logger m_log;
  rclcpp::Node::SharedPtr m_node;

  moveit_cpp::PlanningComponent::PlanRequestParameters m_default_params;
  manipulation_pipeline_interfaces::msg::CartesianLimits m_default_cartesian_limits;

  moveit_cpp::MoveItCppPtr m_moveit_cpp;
  std::shared_ptr<RobotModel> m_robot_model;
  std::shared_ptr<ExecutionPipeline> m_execution_pipeline;
  std::shared_ptr<VisualizationInterface> m_visualization_interface;

  std::shared_ptr<GoalHandle> m_last_goal;
  std::shared_ptr<planning_scene::PlanningScene> m_last_planning_scene;

  WorkerThread<std::shared_ptr<PlanningStep>> m_worker;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_PLANNING_PIPELINE_H_INCLUDED
