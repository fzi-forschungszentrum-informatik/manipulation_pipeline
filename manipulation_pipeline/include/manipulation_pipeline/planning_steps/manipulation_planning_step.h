// Copyright 2026 FZI Forschungszentrum Informatik
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

/*!\file manipulation_pipeline/planning_steps/manipulation_planning_step.h
 * \brief Base for manipulation steps that interact with collision objects
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2026-03-25
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_MANIPULATION_PLANNING_STEPS_H_INCLUDED
#define MANIPULATION_PIPELINE_MANIPULATION_PLANNING_STEPS_H_INCLUDED

#include "manipulation_pipeline/planning_step.h"

#include <Eigen/Geometry>
#include <manipulation_pipeline_interfaces/msg/cartesian_limits.hpp>
#include <manipulation_pipeline_interfaces/msg/linear_motion.hpp>

namespace planning_scene {
class PlanningScene;
} // namespace planning_scene
namespace moveit::core {
class LinkModel;
class JointModelGroup;
} // namespace moveit::core


namespace manipulation_pipeline {

class Planner;
class MarkerInterface;


struct ManipulationPlan
{
  robot_trajectory::RobotTrajectoryPtr ptp;
  robot_trajectory::RobotTrajectoryPtr approach;
  robot_trajectory::RobotTrajectoryPtr retract;
};

class ManipulationPlanningStepBase
{
public:
  virtual ~ManipulationPlanningStepBase() = default;

protected:
  ManipulationPlan
  planManipulation(const Eigen::Isometry3d& target_pose,
                   const moveit::core::LinkModel* tip_link,
                   const moveit::core::LinkModel* reference_link,
                   const moveit::core::JointModelGroup* joint_group,
                   const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
                   const std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                   Planner& planner,
                   MarkerInterface& visualizer,
                   const rclcpp::Logger& log) const;

  virtual std::vector<Eigen::Isometry3d>
  approachWaypoints(const Eigen::Isometry3d& offset) const = 0;
  virtual std::vector<Eigen::Isometry3d>
  retractWaypoints(const Eigen::Isometry3d& offset) const = 0;

  virtual manipulation_pipeline_interfaces::msg::CartesianLimits
  approachLimits(const manipulation_pipeline_interfaces::msg::CartesianLimits& limits) const = 0;
  virtual manipulation_pipeline_interfaces::msg::CartesianLimits
  retractLimits(const manipulation_pipeline_interfaces::msg::CartesianLimits& limits) const = 0;

  virtual void disableCollisions(planning_scene::PlanningScene& planning_scene) const = 0;

  std::vector<Eigen::Isometry3d>
  convertLinearMotion(const manipulation_pipeline_interfaces::msg::LinearMotion& msg,
                      const Eigen::Isometry3d& offset) const;
};

template <typename ActionT>
class ManipulationPlanningStep
  : public ActionPlanningStep<ActionT>
  , public ManipulationPlanningStepBase
{
public:
  ManipulationPlanningStep(const std::shared_ptr<ActionGoalHandle<ActionT>>& goal_handle,
                           std::string name,
                           rclcpp::Logger log);

protected:
  std::vector<Eigen::Isometry3d> approachWaypoints(const Eigen::Isometry3d& offset) const override;
  std::vector<Eigen::Isometry3d> retractWaypoints(const Eigen::Isometry3d& offset) const override;

  manipulation_pipeline_interfaces::msg::CartesianLimits approachLimits(
    const manipulation_pipeline_interfaces::msg::CartesianLimits& limits) const override;
  manipulation_pipeline_interfaces::msg::CartesianLimits retractLimits(
    const manipulation_pipeline_interfaces::msg::CartesianLimits& limits) const override;

  void disableCollisions(planning_scene::PlanningScene& planning_scene) const override;
};

} // namespace manipulation_pipeline


namespace manipulation_pipeline {

template <typename ActionT>
ManipulationPlanningStep<ActionT>::ManipulationPlanningStep(
  const std::shared_ptr<ActionGoalHandle<ActionT>>& goal_handle,
  std::string name,
  rclcpp::Logger log)
  : ActionPlanningStep<ActionT>{goal_handle, std::move(name), std::move(log)}
{
}

template <typename ActionT>
std::vector<Eigen::Isometry3d>
ManipulationPlanningStep<ActionT>::approachWaypoints(const Eigen::Isometry3d& offset) const
{
  return convertLinearMotion(ActionPlanningStep<ActionT>::m_goal->approach, offset);
}

template <typename ActionT>
std::vector<Eigen::Isometry3d>
ManipulationPlanningStep<ActionT>::retractWaypoints(const Eigen::Isometry3d& offset) const
{
  return convertLinearMotion(ActionPlanningStep<ActionT>::m_goal->retract, offset);
}

template <typename ActionT>
manipulation_pipeline_interfaces::msg::CartesianLimits
ManipulationPlanningStep<ActionT>::approachLimits(
  const manipulation_pipeline_interfaces::msg::CartesianLimits& limits) const
{
  return applyCartesianLimits(ActionPlanningStep<ActionT>::m_goal->approach.limits, limits);
}

template <typename ActionT>
manipulation_pipeline_interfaces::msg::CartesianLimits
ManipulationPlanningStep<ActionT>::retractLimits(
  const manipulation_pipeline_interfaces::msg::CartesianLimits& limits) const
{
  return applyCartesianLimits(ActionPlanningStep<ActionT>::m_goal->retract.limits, limits);
}

template <typename ActionT>
void ManipulationPlanningStep<ActionT>::disableCollisions(
  planning_scene::PlanningScene& planning_scene) const
{
  auto& cartesian_planning_scene_acm = planning_scene.getAllowedCollisionMatrixNonConst();
  for (const auto& disabled_collision : ActionPlanningStep<ActionT>::m_goal->disabled_collisions)
  {
    cartesian_planning_scene_acm.setEntry(disabled_collision.link1, disabled_collision.link2, true);
  }
}

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_MANIPULATION_PLANNING_STEPS_H_INCLUDE:D
