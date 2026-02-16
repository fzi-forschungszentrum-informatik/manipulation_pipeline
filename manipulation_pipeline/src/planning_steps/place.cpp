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
 * \date    2025-04-16
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/planning_steps/place.h"

#include "manipulation_pipeline/action.h"
#include "manipulation_pipeline/actions/change_attached_object.h"
#include "manipulation_pipeline/actions/execute_trajectory.h"
#include "manipulation_pipeline/planner.h"
#include "manipulation_pipeline/planning_context.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

Place::Place(const std::shared_ptr<Handle>& handle, rclcpp::Logger log)
  : ActionPlanningStep<Action>{
      handle, fmt::format("Place([{}])", handle->handle().get_goal()->subframe), std::move(log)}
{
}

moveit_cpp::PlanningComponent::PlanRequestParameters Place::applyRequestParams(
  const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params) const
{
  return applyMotionParameters(m_goal->motion_parameters, default_params);
}

std::shared_ptr<ActionSequence>
Place::plan(const RobotModel& robot_model,
            const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
            const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
            PlanningContext& context) const
{
  auto& ee_interface       = robot_model.findEndEffector(m_goal->end_effector);
  auto& planning_interface = ee_interface.planningInterface();
  Planner planner{planning_interface, context, params, limits, m_log};

  // Get attached object
  const auto& attached_body = *getAttachedBody(m_goal->object_name, *context.planning_scene);
  const auto* tip_link      = attached_body.getAttachedLink();

  // Get local target pose
  const auto tip_offset = attached_body.getPose();

  Eigen::Isometry3d target_object_pose_local;
  tf2::convert(m_goal->pose.pose, target_object_pose_local);

  const auto target_pose_local = target_object_pose_local * tip_offset.inverse();

  // Transform poses to reference frame
  if (!context.planning_scene->knowsFrameTransform(m_goal->pose.header.frame_id))
  {
    throw std::runtime_error{
      fmt::format("Unknown target pose frame '{}'", m_goal->pose.header.frame_id)};
  }
  const auto object_frame_transform =
    context.planning_scene->getFrameTransform(m_goal->pose.header.frame_id);

  const auto reference_frame           = planning_interface.referenceLink()->getName();
  const auto reference_frame_transform = context.planning_scene->getFrameTransform(reference_frame);

  const Eigen::Isometry3d local_to_reference_transform =
    reference_frame_transform.inverse() * object_frame_transform;

  const auto target_pose = local_to_reference_transform * target_pose_local;

  // Get approach and retract frames based on target frame
  const double approach_dist = m_goal->approach.distance == 0.0 ? 0.1 : m_goal->approach.distance;
  const auto approach_pose =
    target_pose * Eigen::Isometry3d{Eigen::Translation3d{0, 0, -approach_dist}};

  const double retract_dist = m_goal->retract.distance == 0.0 ? 0.1 : m_goal->retract.distance;
  const auto retract_pose =
    target_pose * Eigen::Isometry3d{Eigen::Translation3d{0, 0, -retract_dist}};

  // Messages for collision object removal
  // If we want to attach the object to a different link afterwards, we need seperate REMOVE and ADD
  // operations
  std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_collision_object_operations;
  attached_collision_object_operations.emplace_back();
  attached_collision_object_operations.back().object.id = attached_body.getName();
  attached_collision_object_operations.back().object.operation =
    moveit_msgs::msg::CollisionObject::REMOVE;

  if (!m_goal->attach_link.empty())
  {
    attached_collision_object_operations.emplace_back();
    attached_collision_object_operations.back().link_name = m_goal->attach_link;
    attached_collision_object_operations.back().object.id = attached_body.getName();
    attached_collision_object_operations.back().object.operation =
      moveit_msgs::msg::CollisionObject::ADD;
  }

  // Do planning inside of IK loop
  robot_trajectory::RobotTrajectoryPtr result_ptp_approach_trajectory;
  robot_trajectory::RobotTrajectoryPtr result_approach_trajectory;
  robot_trajectory::RobotTrajectoryPtr result_retract_trajectory;
  std::shared_ptr<manipulation_pipeline::Action> tool_action;

  // Resolve cartesian limits
  const auto approach_limits = applyCartesianLimits(m_goal->approach.limits, limits);
  const auto retract_limits  = applyCartesianLimits(m_goal->retract.limits, limits);

  // Visualize path
  const auto current_pose = reference_frame_transform.inverse() *
                            context.planning_scene->getFrameTransform(tip_link->getName());
  std::vector path_poses{current_pose, approach_pose, target_pose, retract_pose};
  context.plan_visualizer->addPath(path_poses, reference_frame);
  context.plan_visualizer->publish();

  // Target pose for setFromIk needs to be in model frame
  geometry_msgs::msg::Pose target_pose_msg;
  tf2::convert(reference_frame_transform * target_pose, target_pose_msg);

  const auto initial_state          = context.planning_scene->getCurrentState();
  moveit::core::RobotState ik_state = initial_state;
  if (!ik_state.setFromIK(
        planning_interface.group(),
        target_pose_msg,
        tip_link->getName(),
        0.0,
        [&](moveit::core::RobotState* state,
            const moveit::core::JointModelGroup* group,
            const double* joint_values) -> bool {
          state->setJointGroupPositions(group, joint_values);
          state->update();

          const auto cartesian_planning_scene =
            planning_scene::PlanningScene::clone(context.planning_scene);
          auto& cartesian_planning_scene_acm =
            cartesian_planning_scene->getAllowedCollisionMatrixNonConst();
          for (const auto& disabled_collision : m_goal->disabled_collisions)
          {
            cartesian_planning_scene_acm.setEntry(
              disabled_collision.link1, disabled_collision.link2, true);
          }

          RCLCPP_DEBUG(m_log, "Planning cartesian approach trajectory");
          auto cartesian_approach_trajectory = planner.planCartesian(*state,
                                                                     reference_frame,
                                                                     approach_pose,
                                                                     tip_link,
                                                                     cartesian_planning_scene,
                                                                     &approach_limits);
          if (!cartesian_approach_trajectory || cartesian_approach_trajectory->empty())
          {
            return false;
          }
          cartesian_approach_trajectory->reverse();

          // Clone planning scene in order to properly plan with removed object
          RCLCPP_DEBUG(m_log, "Creating tool action and cloned planning scene");
          const auto detached_planning_scene =
            planning_scene::PlanningScene::clone(context.planning_scene);

          tool_action = createToolAction(ee_interface,
                                         detached_planning_scene->getRobotModel(),
                                         planner,
                                         *state,
                                         m_goal->release_action);
          for (const auto& attached_collision_object_op : attached_collision_object_operations)
          {
            detached_planning_scene->processAttachedCollisionObjectMsg(
              attached_collision_object_op);
          }

          RCLCPP_DEBUG(m_log, "Planning cartesian retract trajectory");
          auto cartesian_retract_trajectory = planner.planCartesian(*state,
                                                                    reference_frame,
                                                                    retract_pose,
                                                                    tip_link,
                                                                    cartesian_planning_scene,
                                                                    &retract_limits);
          if (!cartesian_retract_trajectory)
          {
            return false;
          }

          // Plan ptp motion with original scene (with attached object)
          RCLCPP_DEBUG(m_log, "Planning PTP approach");
          const auto ptp_approach_trajectory =
            planner.plan(initial_state,
                         cartesian_approach_trajectory->getFirstWayPoint(),
                         context.planning_scene);
          if (!ptp_approach_trajectory)
          {
            return false;
          }

          result_ptp_approach_trajectory = ptp_approach_trajectory;
          result_approach_trajectory     = cartesian_approach_trajectory;
          result_retract_trajectory      = cartesian_retract_trajectory;

          result_ptp_approach_trajectory->setWayPointDurationFromPrevious(0, 0.5);
          result_approach_trajectory->setWayPointDurationFromPrevious(0, 0.1);
          result_retract_trajectory->setWayPointDurationFromPrevious(0, 0.1);

          return true;
        }))
  {
    throw std::runtime_error{"Could not find trajectory"};
  }

  // Keep track of current state
  context.planning_scene->setCurrentState(result_retract_trajectory->getLastWayPoint());
  for (const auto& attached_collision_object_op : attached_collision_object_operations)
  {
    context.planning_scene->processAttachedCollisionObjectMsg(attached_collision_object_op);
  }

  auto result = std::make_shared<ActionSequence>(name(), goalHandle());
  result->add(std::make_shared<actions::ExecuteTrajectory>(
    result_ptp_approach_trajectory, m_goal->controller, std::vector{tip_link}, "ptp_approach"));
  result->add(std::make_shared<actions::ExecuteTrajectory>(
    result_approach_trajectory, m_goal->controller, std::vector{tip_link}, "approach"));
  if (tool_action)
  {
    result->add(tool_action);
  }
  for (const auto& attached_collision_object_op : attached_collision_object_operations)
  {
    result->add(
      std::make_shared<actions::ChangeAttachedObject>(attached_collision_object_op, "detach"));
  }
  result->add(std::make_shared<actions::ExecuteTrajectory>(
    result_retract_trajectory, m_goal->controller, std::vector{tip_link}, "retract"));

  return result;

  return std::make_shared<ActionSequence>(name(), goalHandle());
}

const moveit::core::AttachedBody*
Place::getAttachedBody(const std::string& name,
                       const planning_scene::PlanningScene& planning_scene) const
{
  // Get all attached bodies
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  planning_scene.getCurrentState().getAttachedBodies(attached_bodies);

  if (attached_bodies.empty())
  {
    throw std::runtime_error{"There are currently no attached objects"};
  }

  // Try default object
  if (name.empty())
  {
    if (attached_bodies.size() == 1)
    {
      RCLCPP_INFO(
        m_log, "Defaulting to only attached object '%s'", attached_bodies[0]->getName().c_str());
      return attached_bodies[0];
    }
    else
    {
      throw std::runtime_error{fmt::format("Cannot automatically select object to be placed, as "
                                           "there are currently {} attached objects",
                                           attached_bodies.size())};
    }
  }

  // Find object by name
  const auto find_it =
    std::find_if(attached_bodies.begin(), attached_bodies.end(), [&](const auto* attached_body) {
      return attached_body->getName() == m_goal->object_name;
    });
  if (find_it == attached_bodies.end())
  {
    throw std::runtime_error{
      fmt::format("Object '{}' is currently not attached to any link", name)};
  }

  return *find_it;
}

} // namespace manipulation_pipeline
