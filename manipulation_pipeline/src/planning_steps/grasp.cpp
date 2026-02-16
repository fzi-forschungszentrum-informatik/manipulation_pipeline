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
 * \date    2025-04-15
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/planning_steps/grasp.h"

#include "manipulation_pipeline/action.h"
#include "manipulation_pipeline/actions/change_attached_object.h"
#include "manipulation_pipeline/actions/execute_trajectory.h"
#include "manipulation_pipeline/planner.h"
#include "manipulation_pipeline/planning_context.h"

#include <fmt/format.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

Grasp::Grasp(const std::shared_ptr<Handle>& handle, rclcpp::Logger log)
  : ActionPlanningStep<Action>{handle,
                               fmt::format("Grasp({}[{}])",
                                           handle->handle().get_goal()->object_name,
                                           handle->handle().get_goal()->subframe),
                               std::move(log)}
{
}

moveit_cpp::PlanningComponent::PlanRequestParameters Grasp::applyRequestParams(
  const moveit_cpp::PlanningComponent::PlanRequestParameters& default_params) const
{
  return applyMotionParameters(m_goal->motion_parameters, default_params);
}

std::shared_ptr<ActionSequence>
Grasp::plan(const RobotModel& robot_model,
            const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
            const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
            PlanningContext& context) const
{
  auto& ee_interface       = robot_model.findEndEffector(m_goal->end_effector);
  auto& planning_interface = ee_interface.planningInterface();

  Planner planner{planning_interface, context, params, limits, m_log};

  const auto* tip_link = planning_interface.resolveTip(m_goal->tip);

  // Get target pose based on collision object in scene
  const auto collision_object = getCollisionObject(*(context.planning_scene));
  const auto [collision_object_target_pose, subframe_offset] = resolveTargetPose(collision_object);

  const auto target_pose_local = collision_object_target_pose * subframe_offset;

  // Resolve approach and retract poses
  const double approach_dist = m_goal->approach.distance == 0.0 ? 0.1 : m_goal->approach.distance;
  const auto approach_pose_local =
    target_pose_local * Eigen::Isometry3d{Eigen::Translation3d{0, 0, -approach_dist}};

  const double retract_dist = m_goal->retract.distance == 0.0 ? 0.1 : m_goal->retract.distance;
  const auto retract_pose_local =
    target_pose_local * Eigen::Isometry3d{Eigen::Translation3d{0, 0, -retract_dist}};

  // Transform poses into group reference frame
  if (!context.planning_scene->knowsFrameTransform(collision_object.header.frame_id))
  {
    throw std::runtime_error{
      fmt::format("Unknown collision object pose frame '{}'", collision_object.header.frame_id)};
  }
  const auto collision_object_frame_transform =
    context.planning_scene->getFrameTransform(collision_object.header.frame_id);

  const auto reference_frame           = planning_interface.referenceLink()->getName();
  const auto reference_frame_transform = context.planning_scene->getFrameTransform(reference_frame);

  const Eigen::Isometry3d collision_object_to_reference_transform =
    reference_frame_transform.inverse() * collision_object_frame_transform;
  const auto target_pose   = collision_object_to_reference_transform * target_pose_local;
  const auto approach_pose = collision_object_to_reference_transform * approach_pose_local;
  const auto retract_pose  = collision_object_to_reference_transform * retract_pose_local;

  // Resolve cartesian limits
  const auto approach_limits = applyCartesianLimits(m_goal->approach.limits, limits);
  const auto retract_limits  = applyCartesianLimits(m_goal->retract.limits, limits);

  // Visualize path
  const auto current_pose = reference_frame_transform.inverse() *
                            context.planning_scene->getFrameTransform(tip_link->getName());
  std::vector path_poses{current_pose, approach_pose, target_pose, retract_pose};
  context.plan_visualizer->addPath(path_poses, reference_frame);
  context.plan_visualizer->publish();

  // Do planning inside IK loop
  // Goal: Find an IK solution that allows the whole grasp step to succeed
  robot_trajectory::RobotTrajectoryPtr result_ptp_approach_trajectory;
  robot_trajectory::RobotTrajectoryPtr result_approach_trajectory;
  robot_trajectory::RobotTrajectoryPtr result_retract_trajectory;
  std::shared_ptr<manipulation_pipeline::Action> tool_action;

  // Target pose for setFromIk needs to be in model frame
  geometry_msgs::msg::Pose target_pose_msg;
  tf2::convert(reference_frame_transform * target_pose, target_pose_msg);

  // Allow collisions between collision object and end effector
  const auto& ee_links = ee_interface.group()->getLinkModels();
  std::vector<std::string> ee_link_names;
  std::transform(ee_links.begin(),
                 ee_links.end(),
                 std::back_inserter(ee_link_names),
                 [](const auto* link) { return link->getName(); });

  // Create collision object
  moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
  attached_collision_object.object           = collision_object;
  attached_collision_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached_collision_object.link_name        = tip_link->getName();
  attached_collision_object.touch_links      = ee_link_names;

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

          RCLCPP_INFO(m_log, "Planning cartesian approach trajectory");
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

          // Clone planning scene in order to properly handle attached object
          RCLCPP_INFO(m_log, "Creating tool action and cloned planning scene");
          const auto attached_planning_scene =
            planning_scene::PlanningScene::clone(context.planning_scene);

          tool_action = createToolAction(ee_interface,
                                         attached_planning_scene->getRobotModel(),
                                         planner,
                                         *state,
                                         m_goal->grasp_action);
          attached_planning_scene->processAttachedCollisionObjectMsg(attached_collision_object);

          RCLCPP_INFO(m_log, "Planning cartesian retract trajectory");
          // TODO: Why does this not work with attached_planning_scene instead of
          // context.planning_scene?
          auto cartesian_retract_trajectory = planner.planCartesian(*state,
                                                                    reference_frame,
                                                                    retract_pose,
                                                                    tip_link,
                                                                    cartesian_planning_scene,
                                                                    &retract_limits);
          // planner.planCartesian(*state, retract_pose, tip_link, attached_planning_scene);
          if (!cartesian_retract_trajectory)
          {
            return false;
          }

          // Plan ptp motion with original scene (without attached object)
          RCLCPP_INFO(m_log, "Planning PTP approach");
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
  context.planning_scene->processAttachedCollisionObjectMsg(attached_collision_object);

  // Create action sequence
  auto result = std::make_shared<ActionSequence>(name(), goalHandle());
  result->add(std::make_shared<actions::ExecuteTrajectory>(
    result_ptp_approach_trajectory, m_goal->controller, std::vector{tip_link}, "ptp_approach"));
  result->add(std::make_shared<actions::ExecuteTrajectory>(
    result_approach_trajectory, m_goal->controller, std::vector{tip_link}, "approach"));
  if (tool_action)
  {
    result->add(tool_action);
  }
  result->add(std::make_shared<actions::ChangeAttachedObject>(attached_collision_object, "attach"));
  result->add(std::make_shared<actions::ExecuteTrajectory>(
    result_retract_trajectory, m_goal->controller, std::vector{tip_link}, "retract"));

  return result;
} // namespace manipulation_pipeline

moveit_msgs::msg::CollisionObject
Grasp::getCollisionObject(const planning_scene::PlanningScene& planning_scene) const
{
  const auto& object_name = m_goal->object_name;

  moveit_msgs::msg::CollisionObject obj;
  if (planning_scene.getCollisionObjectMsg(obj, object_name))
  {
    return obj;
  }
  else if (moveit_msgs::msg::AttachedCollisionObject attached_obj;
           planning_scene.getAttachedCollisionObjectMsg(attached_obj, object_name))
  {
    return attached_obj.object;
  }

  throw std::runtime_error{fmt::format("No object named '{}'", object_name)};
}

std::pair<Eigen::Isometry3d, Eigen::Isometry3d>
Grasp::resolveTargetPose(const moveit_msgs::msg::CollisionObject& object) const
{
  const auto& subframe = m_goal->subframe;

  Eigen::Isometry3d pose;
  tf2::convert(object.pose, pose);
  Eigen::Isometry3d subframe_offset = Eigen::Isometry3d::Identity();

  if (subframe.empty())
  {
    return std::make_pair(pose, subframe_offset);
  }

  if (object.subframe_names.size() != object.subframe_poses.size())
  {
    throw std::runtime_error{
      fmt::format("Invalid subframe pose layout of collision object '{}'", object.id)};
  }

  const auto& subframe_names = object.subframe_names;
  if (const auto subframe_it = std::find(subframe_names.begin(), subframe_names.end(), subframe);
      subframe_it != subframe_names.end())
  {
    const auto i = subframe_it - subframe_names.begin();

    tf2::convert(object.subframe_poses[i], subframe_offset);
    return std::make_pair(pose, subframe_offset);
  }
  else
  {
    throw std::runtime_error{
      fmt::format("Collision object '{}' has no subframe '{}'", object.id, subframe)};
  }
}

} // namespace manipulation_pipeline
