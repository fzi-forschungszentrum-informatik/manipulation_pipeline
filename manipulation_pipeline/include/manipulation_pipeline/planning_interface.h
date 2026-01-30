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

/*!\file manipulation_pipeline/planning_interface.h
 * \brief ROS interfaces to planning pipeline
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-08
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_PLANNING_INTERFACE_H_INCLUDED
#define MANIPULATION_PIPELINE_PLANNING_INTERFACE_H_INCLUDED

#include "goal_handle.h"
#include "planning_pipeline.h"

#include <manipulation_pipeline_interfaces/action/actuate_tool.hpp>
#include <manipulation_pipeline_interfaces/action/execute_path.hpp>
#include <manipulation_pipeline_interfaces/action/grasp.hpp>
#include <manipulation_pipeline_interfaces/action/move_to_configuration.hpp>
#include <manipulation_pipeline_interfaces/action/move_to_named_pose.hpp>
#include <manipulation_pipeline_interfaces/action/move_to_pose.hpp>
#include <manipulation_pipeline_interfaces/action/place.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace manipulation_pipeline {

/*! \brief Provide ROS action interfaces for interacting with the planning pipeline
 */
class PlanningInterface
{
public:
  PlanningInterface(std::shared_ptr<PlanningPipeline> planning_pipeline,
                    const rclcpp::Node::SharedPtr& node,
                    rclcpp::Logger log);

private:
  using MoveToNamedPoseAction     = manipulation_pipeline_interfaces::action::MoveToNamedPose;
  using MoveToPoseAction          = manipulation_pipeline_interfaces::action::MoveToPose;
  using MoveToConfigurationAction = manipulation_pipeline_interfaces::action::MoveToConfiguration;
  using GraspAction               = manipulation_pipeline_interfaces::action::Grasp;
  using PlaceAction               = manipulation_pipeline_interfaces::action::Place;
  using ActuateToolAction         = manipulation_pipeline_interfaces::action::ActuateTool;
  using ExecutePathAction         = manipulation_pipeline_interfaces::action::ExecutePath;

  template <typename ActionT>
  rclcpp_action::GoalResponse actionGoalCb(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const typename ActionT::Goal> goal);
  template <typename ActionT, typename PlanningStepT>
  void actionAcceptCb(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);
  template <typename ActionT>
  rclcpp_action::CancelResponse
  actionCancelCb(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);

  rclcpp::Logger m_log;

  std::shared_ptr<PlanningPipeline> m_planning_pipeline;
  std::size_t m_goal_cnt;

  rclcpp_action::Server<MoveToNamedPoseAction>::SharedPtr m_move_to_named_pose;
  rclcpp_action::Server<MoveToPoseAction>::SharedPtr m_move_to_pose;
  rclcpp_action::Server<MoveToConfigurationAction>::SharedPtr m_move_to_configuration;
  rclcpp_action::Server<GraspAction>::SharedPtr m_grasp;
  rclcpp_action::Server<PlaceAction>::SharedPtr m_place;
  rclcpp_action::Server<ActuateToolAction>::SharedPtr m_actuate_tool;
  rclcpp_action::Server<ExecutePathAction>::SharedPtr m_execute_path;
};

} // namespace manipulation_pipeline


namespace manipulation_pipeline {

template <typename ActionT>
rclcpp_action::GoalResponse
PlanningInterface::actionGoalCb(const rclcpp_action::GoalUUID& uuid,
                                std::shared_ptr<const typename ActionT::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ActionT, typename PlanningStepT>
void PlanningInterface::actionAcceptCb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
{
  const auto planning_step = std::make_shared<PlanningStepT>(
    std::make_shared<ActionGoalHandle<ActionT>>(goal_handle, m_goal_cnt++), m_log);
  m_planning_pipeline->addPlanningStep(planning_step);
}

template <typename ActionT>
rclcpp_action::CancelResponse PlanningInterface::actionCancelCb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
{
  return rclcpp_action::CancelResponse::REJECT;
}

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_PLANNING_INTERFACE_H_INCLUDED
