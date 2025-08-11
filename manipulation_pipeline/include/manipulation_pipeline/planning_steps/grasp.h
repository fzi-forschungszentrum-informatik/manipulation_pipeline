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

/*!\file manipulation_pipeline/planning_steps/grasp.h
 * \brief Grasp a collision object
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-15
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_GRASP_H_INCLUDED
#define MANIPULATION_PIPELINE_GRASP_H_INCLUDED

#include "manipulation_pipeline/planning_step.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <manipulation_pipeline_interfaces/action/grasp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <optional>
#include <rclcpp_action/server_goal_handle.hpp>

namespace manipulation_pipeline {

/*! \brief Move to and grasp a collision object
 */
class Grasp : public ActionPlanningStep<manipulation_pipeline_interfaces::action::Grasp>
{
public:
  using Action = manipulation_pipeline_interfaces::action::Grasp;
  using Handle = ActionGoalHandle<Action>;

  Grasp(const std::shared_ptr<Handle>& handle, rclcpp::Logger log);

  [[nodiscard]] moveit_cpp::PlanningComponent::PlanRequestParameters applyRequestParams(
    const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params) const override;

  std::shared_ptr<ActionSequence>
  plan(const RobotModel& robot_model,
       const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
       const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
       PlanningContext& context) const override;

private:
  moveit_msgs::msg::CollisionObject
  getCollisionObject(const planning_scene::PlanningScene& planning_scene) const;

  geometry_msgs::msg::PoseStamped
  resolveTargetPose(const moveit_msgs::msg::CollisionObject& object) const;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_GRASP_H_INCLUDED
