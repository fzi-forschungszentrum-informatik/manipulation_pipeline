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
#include "manipulation_pipeline/action.h"

#include "manipulation_pipeline/goal_handle.h"
#include "manipulation_pipeline/visualization_interface.h"

#include <fmt/format.h>
#include <manipulation_pipeline_interfaces/msg/progress.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/logging.hpp>

namespace manipulation_pipeline {

Action::Action(std::string name)
  : m_name{std::move(name)}
{
}

const std::string& Action::name() const
{
  return m_name;
}

void Action::visualize(const std::shared_ptr<MarkerInterface>& marker_interface,
                       GoalHandle& goal_handle) const
{
}

ActionSequence::ActionSequence(std::string name, std::shared_ptr<GoalHandle> goal_handle)
  : m_name{std::move(name)}
  , m_goal_handle{std::move(goal_handle)}
{
}

ActionSequence::ActionSequence(std::string name,
                               std::shared_ptr<Action> first_action,
                               std::shared_ptr<GoalHandle> goal_handle)
  : m_name{std::move(name)}
  , m_goal_handle{std::move(goal_handle)}
{
  m_actions.push_back(std::move(first_action));
}

const std::string& ActionSequence::name() const
{
  return m_name;
}

const std::shared_ptr<GoalHandle>& ActionSequence::goalHandle() const
{
  return m_goal_handle;
}

void ActionSequence::add(std::shared_ptr<Action> action)
{
  m_actions.push_back(std::move(action));
}

void ActionSequence::execute(moveit_cpp::MoveItCpp& moveit_cpp, rclcpp::Logger log)
{
  RCLCPP_INFO(log, "Executing actions for goal %s", m_name.c_str());

  for (std::size_t i = 0; i < m_actions.size(); ++i)
  {
    const auto action_str = fmt::format("Executing action {} ({}/{}) of goal {}",
                                        m_actions[i]->name(),
                                        i + 1,
                                        m_actions.size(),
                                        m_name);

    RCLCPP_INFO(log, "%s", action_str.c_str());

    m_goal_handle->feedback(manipulation_pipeline_interfaces::msg::Progress::PROGRESS_EXECUTING,
                            action_str,
                            i,
                            m_actions.size());
    m_actions[i]->execute(moveit_cpp, log);
  }

  RCLCPP_INFO(log, "Finished executing actions for goal %s", m_name.c_str());
}

void ActionSequence::visualize(const std::shared_ptr<MarkerInterface>& marker_interface) const
{
  for (const auto& action : m_actions)
  {
    action->visualize(marker_interface, *m_goal_handle);
  }
  marker_interface->publish();
}

} // namespace manipulation_pipeline
