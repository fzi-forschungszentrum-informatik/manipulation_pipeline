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

/*!\file manipulation_pipeline/planner.h
 * \brief Interface to MoveIt planner functionality
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-15
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_PLANNER_H_INCLUDED
#define MANIPULATION_PIPELINE_PLANNER_H_INCLUDED

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <manipulation_pipeline_interfaces/msg/cartesian_limits.hpp>
#include <memory>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <pilz_industrial_motion_planner/command_list_manager.hpp>
#include <rclcpp/logger.hpp>
#include <string>

namespace planning_scene {
class PlanningScene;
}

namespace manipulation_pipeline {

class GroupInterface;
class PlanningContext;

/*! \brief Interface to MoveIt planner functionality for single trajectories
 */
class Planner
{
public:
  Planner(const GroupInterface& group_interface,
          const PlanningContext& planning_context,
          const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
          const manipulation_pipeline_interfaces::msg::CartesianLimits& cartesian_limits,
          rclcpp::Logger log);

  [[nodiscard]] const std::string& lastErrorMsg() const;

  robot_trajectory::RobotTrajectoryPtr plan(const moveit::core::RobotState& initial_state,
                                            const std::string& target_pose,
                                            const planning_scene::PlanningScenePtr& planning_scene);

  robot_trajectory::RobotTrajectoryPtr plan(const moveit::core::RobotState& initial_state,
                                            const moveit::core::RobotState& target_state,
                                            const planning_scene::PlanningScenePtr& planning_scene);

  robot_trajectory::RobotTrajectoryPtr plan(const moveit::core::RobotState& initial_state,
                                            const geometry_msgs::msg::PoseStamped& target_pose,
                                            const moveit::core::LinkModel* tip,
                                            const planning_scene::PlanningScenePtr& planning_scene);

  robot_trajectory::RobotTrajectoryPtr
  planCartesian(const moveit::core::RobotState& initial_state,
                const Eigen::Isometry3d& target_pose,
                const moveit::core::LinkModel* tip,
                const planning_scene::PlanningScenePtr& planning_scene,
                const manipulation_pipeline_interfaces::msg::CartesianLimits* limits = nullptr);

  robot_trajectory::RobotTrajectoryPtr planCartesianSequence(
    const moveit::core::RobotState& initial_state,
    const std::string& waypoint_frame,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const moveit::core::LinkModel* tip,
    const planning_scene::PlanningScenePtr& planning_scene,
    const manipulation_pipeline_interfaces::msg::CartesianLimits* limits = nullptr);

private:
  robot_trajectory::RobotTrajectoryPtr
  doPlan(const moveit_cpp::PlanningComponent::PlanRequestParameters& params,
         const planning_scene::PlanningScenePtr& planning_scene);

  rclcpp::Logger m_log;
  rclcpp::Clock::SharedPtr m_clock;

  const moveit::core::JointModelGroup* m_group;
  moveit_cpp::PlanningComponent* m_planning_component;

  pilz_industrial_motion_planner::CommandListManager m_command_list_manager;

  moveit_cpp::PlanningComponent::PlanRequestParameters m_params;
  manipulation_pipeline_interfaces::msg::CartesianLimits m_cartesian_limits;

  std::optional<std::string> m_last_error;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_PLANNER_H_INCLUDED
