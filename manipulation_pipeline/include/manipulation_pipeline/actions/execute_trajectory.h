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

/*!\file manipulation_pipeline/actions/execute_trajectory.h
 * \brief Execute a planned trajectory
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-16
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_EXECUTE_TRAJECTORY_H_INCLUDED
#define MANIPULATION_PIPELINE_EXECUTE_TRAJECTORY_H_INCLUDED

#include "manipulation_pipeline/action.h"

#include <memory>

namespace moveit::core {
class LinkModel;
}
namespace robot_trajectory {
class RobotTrajectory;
}

namespace manipulation_pipeline::actions {

/*! \brief Execute a previously planned trajectory
 */
class ExecuteTrajectory : public Action
{
public:
  ExecuteTrajectory(std::shared_ptr<robot_trajectory::RobotTrajectory> trajectory,
                    const std::string& controller,
                    const std::vector<const moveit::core::LinkModel*> tip_links,
                    std::string name);

  void execute(moveit_cpp::MoveItCpp& moveit_cpp, rclcpp::Logger& log) const override;
  void visualize(const std::shared_ptr<MarkerInterface>& marker_interface,
                 GoalHandle& goal_handle) const override;

private:
  std::shared_ptr<robot_trajectory::RobotTrajectory> m_trajectory;
  std::string m_controller;

  std::vector<const moveit::core::LinkModel*> m_tip_links;
};

} // namespace manipulation_pipeline::actions

#endif // MANIPULATION_PIPELINE_EXECUTE_TRAJECTORY_H_INCLUDED
