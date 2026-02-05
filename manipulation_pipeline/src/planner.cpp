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
#include "manipulation_pipeline/planner.h"

#include "manipulation_pipeline/planning_context.h"
#include "manipulation_pipeline/robot_model.h"

#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <pilz_industrial_motion_planner/command_list_manager.hpp>
#include <pilz_industrial_motion_planner/trajectory_generator_lin.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

Planner::Planner(const GroupInterface& group_interface,
                 const PlanningContext& planning_context,
                 const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
                 const manipulation_pipeline_interfaces::msg::CartesianLimits& cartesian_limits,
                 rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_clock{planning_context.node->get_clock()}
  , m_group{group_interface.group()}
  , m_planning_component{&group_interface.planningComponent()}
  , m_time_parametrization{std::make_unique<
      trajectory_processing::TimeOptimalTrajectoryGeneration>()}
  , m_command_list_manager{planning_context.node, planning_context.planning_scene->getRobotModel()}
  , m_params{params}
  , m_cartesian_limits{cartesian_limits}
{
}

const std::string& Planner::lastErrorMsg() const
{
  if (!m_last_error)
  {
    throw std::runtime_error{"No error message recorded"};
  }

  return *m_last_error;
}

std::shared_ptr<robot_trajectory::RobotTrajectory>
Planner::plan(const moveit::core::RobotState& initial_state,
              const std::string& target_pose,
              const planning_scene::PlanningScenePtr& planning_scene)
{
  RCLCPP_INFO(m_log,
              "Planning motion to named pose '%s' (group '%s')",
              target_pose.c_str(),
              m_group->getName().c_str());

  const auto& named_target_states = m_planning_component->getNamedTargetStates();
  if (const auto find_it =
        std::find(named_target_states.begin(), named_target_states.end(), target_pose);
      find_it == named_target_states.end())
  {
    throw std::runtime_error{fmt::format("There is no named state '{}' for group '{}'",
                                         target_pose,
                                         m_planning_component->getPlanningGroupName())};
  }

  m_planning_component->setStartState(initial_state);
  m_planning_component->setGoal(target_pose);

  return doPlan(m_params, planning_scene);
}

std::shared_ptr<robot_trajectory::RobotTrajectory>
Planner::plan(const moveit::core::RobotState& initial_state,
              const moveit::core::RobotState& target_state,
              const planning_scene::PlanningScenePtr& planning_scene)
{
  RCLCPP_INFO(
    m_log, "Planning free-space motion to configuration (group '%s')", m_group->getName().c_str());

  m_planning_component->setStartState(initial_state);
  m_planning_component->setGoal(target_state);

  return doPlan(m_params, planning_scene);
}

robot_trajectory::RobotTrajectoryPtr
Planner::plan(const moveit::core::RobotState& initial_state,
              const geometry_msgs::msg::PoseStamped& target_pose,
              const moveit::core::LinkModel* tip,
              const planning_scene::PlanningScenePtr& planning_scene)
{
  RCLCPP_INFO(
    m_log,
    "Planning free-space motion to pose (%.2f, %.2f, %.2f)[%.2f, %.2f, %.2f, %.2f] (frame='%s', "
    "tip='%s', group '%s')",
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z,
    target_pose.pose.orientation.x,
    target_pose.pose.orientation.y,
    target_pose.pose.orientation.z,
    target_pose.pose.orientation.w,
    target_pose.header.frame_id.c_str(),
    tip->getName().c_str(),
    m_group->getName().c_str());

  m_planning_component->setStartState(initial_state);
  m_planning_component->setGoal(target_pose, tip->getName());

  return doPlan(m_params, planning_scene);
}

robot_trajectory::RobotTrajectoryPtr
Planner::planCartesian(const moveit::core::RobotState& initial_state,
                       const Eigen::Isometry3d& target_pose,
                       const moveit::core::LinkModel* tip,
                       const planning_scene::PlanningScenePtr& planning_scene,
                       const manipulation_pipeline_interfaces::msg::CartesianLimits* limits)
{
  geometry_msgs::msg::Pose target_pose_msg;
  tf2::convert(target_pose, target_pose_msg);

  return planCartesianSequence(initial_state,
                               initial_state.getRobotModel()->getModelFrame(),
                               {target_pose_msg},
                               tip,
                               planning_scene,
                               limits);
}

robot_trajectory::RobotTrajectoryPtr
Planner::planCartesianSequence(const moveit::core::RobotState& initial_state,
                               const std::string& waypoint_frame,
                               const std::vector<geometry_msgs::msg::Pose>& waypoints,
                               const moveit::core::LinkModel* tip,
                               const planning_scene::PlanningScenePtr& planning_scene,
                               const manipulation_pipeline_interfaces::msg::CartesianLimits* limits)
{
  RCLCPP_INFO(m_log, "Planning cartesian motion along %zu waypoints", waypoints.size());
  if (waypoints.empty())
  {
    return robot_trajectory::RobotTrajectoryPtr{};
  }

  // Extract joint limits
  pilz_industrial_motion_planner::JointLimitsContainer pilz_joint_limits;
  for (auto joint_model : initial_state.getRobotModel()->getJointModels())
  {
    pilz_industrial_motion_planner::JointLimit joint_limit;

    const auto& bounds = joint_model->getVariableBounds();

    if (bounds.size() == 1)
    {
      joint_limit.has_position_limits     = bounds[0].position_bounded_;
      joint_limit.min_position            = bounds[0].min_position_;
      joint_limit.max_position            = bounds[0].max_position_;
      joint_limit.has_velocity_limits     = bounds[0].velocity_bounded_;
      joint_limit.max_velocity            = bounds[0].max_velocity_;
      joint_limit.has_acceleration_limits = bounds[0].acceleration_bounded_;
      joint_limit.max_acceleration        = bounds[0].max_acceleration_;
      joint_limit.has_deceleration_limits = bounds[0].acceleration_bounded_;
      joint_limit.max_deceleration        = -bounds[0].max_acceleration_;
      pilz_joint_limits.addLimit(joint_model->getName(), joint_limit);
    }
  }

  if (!limits)
  {
    limits = &m_cartesian_limits;
  }

  cartesian_limits::Params pilz_cartesian_limits;
  pilz_cartesian_limits.max_trans_vel = limits->max_trans_vel;
  pilz_cartesian_limits.max_trans_acc = limits->max_trans_acc;
  pilz_cartesian_limits.max_trans_dec = limits->max_trans_dec;
  pilz_cartesian_limits.max_rot_vel   = limits->max_rot_vel;

  pilz_industrial_motion_planner::LimitsContainer pilz_limits;
  pilz_limits.setCartesianLimits(pilz_cartesian_limits);
  pilz_limits.setJointLimits(pilz_joint_limits);

  // Create trajectory generator
  // Unfortunately we cannot use the pilz planner directly, as cartesian limits are only
  // configurable through parameters there.
  pilz_industrial_motion_planner::TrajectoryGeneratorLIN trajectory_generator{
    planning_scene->getRobotModel(), pilz_limits, m_group->getName()};

  std::vector<robot_trajectory::RobotTrajectoryPtr> trajectories;

  moveit::core::RobotStatePtr start_state =
    std::make_shared<moveit::core::RobotState>(initial_state);

  geometry_msgs::msg::PoseStamped waypoint_pose;
  waypoint_pose.header.frame_id = waypoint_frame;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    RCLCPP_INFO(m_log, "Planning waypoint %zu", i);

    waypoint_pose.pose = waypoints[i];

    // Generate general request
    moveit_msgs::msg::MotionPlanRequest req;
    req.group_name                      = m_group->getName();
    req.pipeline_id                     = "pilz_industrial_motion_planner";
    req.planner_id                      = "LIN";
    req.num_planning_attempts           = m_params.planning_attempts;
    req.allowed_planning_time           = m_params.planning_time;
    req.max_velocity_scaling_factor     = 1.0;
    req.max_acceleration_scaling_factor = 1.0;

    // Create goal constraint from waypoint
    moveit_msgs::msg::Constraints waypoint_constraint;
    moveit_msgs::msg::PositionConstraint pos_constraint;
    pos_constraint.header.frame_id = waypoint_frame;
    pos_constraint.link_name       = tip->getName();

    req.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(tip->getName(), waypoint_pose));

    // Set start state
    start_state->update();
    moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);

    // Generate LIN segment
    planning_interface::MotionPlanResponse res;
    trajectory_generator.generate(planning_scene, req, res);
    if (res.error_code != res.error_code.SUCCESS)
    {
      return robot_trajectory::RobotTrajectoryPtr{};
    }
    trajectories.push_back(res.trajectory);
    start_state = res.trajectory->getLastWayPointPtr();
  }


  /*auto trajectories =
    m_command_list_manager.solve(cloned_planning_scene, m_pilz_pipeline, sequence_req);*/

  auto& result = trajectories[0];
  for (std::size_t i = 1; i < trajectories.size(); ++i)
  {
    result->append(*trajectories[i], 0.1);
  }

  return result;
}

void Planner::retimeTrajectory(robot_trajectory::RobotTrajectory& trajectory) const
{
  const double initial_duration = trajectory.getDuration();
  if (!m_time_parametrization->computeTimeStamps(trajectory))
  {
    throw std::runtime_error{"Unable to retime trajectory"};
  }
  RCLCPP_INFO(
    m_log, "Retimed trajectory from %.02fs to %.02fs", initial_duration, trajectory.getDuration());
}

robot_trajectory::RobotTrajectoryPtr
Planner::doPlan(const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
                const planning_scene::PlanningScenePtr& planning_scene)
{
  m_last_error = std::nullopt;

  const auto time_start = m_clock->now();
  const auto time_until = time_start + rclcpp::Duration::from_seconds(params.planning_time);

  moveit_cpp::PlanningComponent::PlanRequestParameters cur_params = params;

  for (std::size_t i = 0;; ++i)
  {
    const auto now = m_clock->now();
    if ((time_until - now).seconds() < (0.1 * params.planning_time))
    {
      m_last_error = fmt::format(
        "Not able to plan after {} attempts (timeout: {})", (i + 1), params.planning_time);
      return robot_trajectory::RobotTrajectoryPtr{};
    }

    cur_params.planning_time = (time_until - now).seconds();

    RCLCPP_INFO(
      m_log, "Planning attempt %zu with timeout %0.2fs", (i + 1), cur_params.planning_time);
    const auto result = m_planning_component->plan(cur_params, planning_scene);
    if (result)
    {
      RCLCPP_INFO(
        m_log, "Planned trajectory in attempt %zu within %.02fs", (i + 1), result.planning_time);
      return result.trajectory;
    }
    else if (result.error_code == moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN)
    {
      RCLCPP_INFO(m_log, "Unable to plan, new attempt");
    }
    else
    {
      m_last_error = moveit::core::errorCodeToString(result.error_code);
      return robot_trajectory::RobotTrajectoryPtr{};
    }
  }
}

} // namespace manipulation_pipeline
