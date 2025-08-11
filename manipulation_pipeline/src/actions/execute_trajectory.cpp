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
#include "manipulation_pipeline/actions/execute_trajectory.h"

#include "manipulation_pipeline/goal_handle.h"
#include "manipulation_pipeline/robot_model.h"
#include "manipulation_pipeline/visualization_interface.h"

#include <fmt/format.h>
#include <moveit/moveit_cpp/moveit_cpp.hpp>

namespace manipulation_pipeline::actions {

ExecuteTrajectory::ExecuteTrajectory(std::shared_ptr<robot_trajectory::RobotTrajectory> trajectory,
                                     const std::string& controller,
                                     const std::vector<const moveit::core::LinkModel*> tip_links,
                                     std::string name)
  : Action{std::move(name)}
  , m_trajectory{std::move(trajectory)}
  , m_controller{controller}
  , m_tip_links{tip_links}
{
}

void ExecuteTrajectory::execute(moveit_cpp::MoveItCpp& moveit_cpp, rclcpp::Logger& log) const
{
  std::vector<std::string> controllers;
  if (!m_controller.empty())
  {
    controllers.push_back(m_controller);
  }

  const auto result = moveit_cpp.execute(m_trajectory, controllers);
  if (!result)
  {
    throw std::runtime_error{fmt::format("Execution error: {}", result.asString())};
  }
}

void ExecuteTrajectory::visualize(const std::shared_ptr<MarkerInterface>& marker_interface,
                                  GoalHandle& goal_handle) const
{
  const auto trajectory_marker_ids = marker_interface->addTrajectory(*m_trajectory, m_tip_links);

  goal_handle.addDoneCb([this, trajectory_marker_ids, marker_interface](bool success) {
    if (success)
    {
      for (std::size_t id : trajectory_marker_ids)
      {
        marker_interface->remove(id);
      }
    }
  });
}

} // namespace manipulation_pipeline::actions
