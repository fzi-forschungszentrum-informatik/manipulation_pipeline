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

/*!\file
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2026-03-25
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/planning_steps/manipulation_planning_step.h"

#include "manipulation_pipeline/ik_sampler.h"
#include "manipulation_pipeline/planner.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

ManipulationPlan ManipulationPlanningStepBase::planManipulation(
  const Eigen::Isometry3d& target_pose,
  const moveit::core::LinkModel* tip_link,
  const moveit::core::LinkModel* reference_link,
  const moveit::core::JointModelGroup* joint_group,
  const manipulation_pipeline_interfaces::msg::CartesianLimits& limits,
  const moveit_msgs::msg::AttachedCollisionObject& collision_object,
  const std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
  Planner& planner,
  MarkerInterface& visualizer,
  const rclcpp::Logger& log) const
{
  geometry_msgs::msg::Pose target_pose_msg;
  tf2::convert(target_pose, target_pose_msg);
  RCLCPP_INFO(log,
              "Planning manipulation with target pose [%.05f, %.05f, %.05f](%.02f, %.02f, %.02f, "
              "%.02f) (frame=%s) and tip link %s",
              target_pose_msg.position.x,
              target_pose_msg.position.y,
              target_pose_msg.position.z,
              target_pose_msg.orientation.x,
              target_pose_msg.orientation.y,
              target_pose_msg.orientation.z,
              target_pose_msg.orientation.w,
              reference_link->getName().c_str(),
              tip_link->getName().c_str());

  const auto reference_frame_transform =
    planning_scene->getFrameTransform(reference_link->getName());

  // Sample IK solutions
  IkSampler sampler{planning_scene->getCurrentState(),
                    tip_link,
                    joint_group,
                    reference_frame_transform * target_pose,
                    50,
                    log};
  const auto ik_samples = sampler.sampleAll();

  // Get approach and retract waypoints and limits
  auto approach_waypoints      = approachWaypoints(target_pose);
  const auto retract_waypoints = retractWaypoints(target_pose);

  const auto approach_limits = approachLimits(limits);
  const auto retract_limits  = retractLimits(limits);

  // Visualize plan
  visualizePlan(reference_frame_transform.inverse() *
                  planning_scene->getFrameTransform(tip_link->getName()),
                approach_waypoints,
                target_pose,
                retract_waypoints,
                reference_link->getName(),
                visualizer);

  // Prepare planning scenes
  const auto cartesian_planning_scene = planning_scene::PlanningScene::clone(planning_scene);
  disableCollisions(*cartesian_planning_scene);

  const auto attached_planning_scene =
    planning_scene::PlanningScene::clone(cartesian_planning_scene);
  attached_planning_scene->processAttachedCollisionObjectMsg(collision_object);

  // Invert approach as we plan starting from ik sample
  std::reverse(approach_waypoints.begin(), approach_waypoints.end()); // We plan in reverse

  // Calculate trajectories for approach and retract
  std::vector<robot_trajectory::RobotTrajectoryPtr> approach_trajectories;
  std::vector<robot_trajectory::RobotTrajectoryPtr> retract_trajectories;
  for (std::size_t i = 0; i < ik_samples.size(); ++i)
  {
    const auto& state = ik_samples[i];

    RCLCPP_INFO(log, "Planning approach for sample %zu/%zu", i + 1, ik_samples.size());
    auto approach_trajectory = planner.planCartesianSequence(state,
                                                             reference_link->getName(),
                                                             approach_waypoints,
                                                             tip_link,
                                                             cartesian_planning_scene,
                                                             &approach_limits);
    if (!approach_trajectory || approach_trajectory->empty())
    {
      continue;
    }
    approach_trajectory->reverse();

    RCLCPP_INFO(log, "Planning retract for sample %zu/%zu", i + 1, ik_samples.size());
    auto retract_trajectory = planner.planCartesianSequence(state,
                                                            reference_link->getName(),
                                                            retract_waypoints,
                                                            tip_link,
                                                            attached_planning_scene,
                                                            &retract_limits);
    if (!retract_trajectory)
    {
      continue;
    }

    approach_trajectories.push_back(std::move(approach_trajectory));
    retract_trajectories.push_back(std::move(retract_trajectory));
  }
  RCLCPP_INFO(log,
              "Calculated %zu possible cartesian paths for %zu samples",
              approach_trajectories.size(),
              ik_samples.size());

  // Plan PTP trajectory
  std::vector<moveit::core::RobotState> ptp_goal_states;
  ptp_goal_states.reserve(approach_trajectories.size());
  std::transform(approach_trajectories.begin(),
                 approach_trajectories.end(),
                 std::back_inserter(ptp_goal_states),
                 [&](const auto& t) { return t->getFirstWayPoint(); });

  RCLCPP_INFO(log, "Planning PTP trajectory");
  auto ptp_trajectory =
    planner.plan(planning_scene->getCurrentState(), ptp_goal_states, planning_scene);
  if (!ptp_trajectory)
  {
    throw std::runtime_error{"Unable to plan ptp trajectory"};
  }

  // Figure out corresponding approach and retract index
  std::vector<double> distances;
  distances.reserve(approach_trajectories.size());
  for (std::size_t i = 0; i < approach_trajectories.size(); ++i)
  {
    distances.push_back(
      ptp_trajectory->getLastWayPoint().distance(approach_trajectories[i]->getFirstWayPoint()));
  }
  const auto closest_i = std::min_element(distances.begin(), distances.end()) - distances.begin();
  auto& approach_trajectory = approach_trajectories[closest_i];
  auto& retract_trajectory  = retract_trajectories[closest_i];
  RCLCPP_INFO(log, "PTP trajectory corresponds to approach/retract %zu", closest_i);

  // Create result
  ptp_trajectory->setWayPointDurationFromPrevious(0, 0.5);
  approach_trajectory->setWayPointDurationFromPrevious(0, 0.1);
  retract_trajectory->setWayPointDurationFromPrevious(0, 0.1);
  return ManipulationPlan{
    std::move(ptp_trajectory), std::move(approach_trajectory), std::move(retract_trajectory)};
}

std::vector<Eigen::Isometry3d> ManipulationPlanningStepBase::convertLinearMotion(
  const manipulation_pipeline_interfaces::msg::LinearMotion& msg,
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

void ManipulationPlanningStepBase::visualizePlan(
  const Eigen::Isometry3d& inital_pose,
  const std::vector<Eigen::Isometry3d>& approach_waypoints,
  const Eigen::Isometry3d& target_pose,
  const std::vector<Eigen::Isometry3d>& retract_waypoints,
  const std::string& reference_frame,
  MarkerInterface& visualizer) const
{
  std::vector poses{inital_pose};
  std::copy(approach_waypoints.begin(), approach_waypoints.end(), std::back_inserter(poses));
  poses.push_back(target_pose);
  std::copy(retract_waypoints.begin(), retract_waypoints.end(), std::back_inserter(poses));

  visualizer.addPath(poses, reference_frame);
  visualizer.publish();
}

} // namespace manipulation_pipeline
