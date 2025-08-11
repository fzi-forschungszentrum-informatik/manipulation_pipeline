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

/*!\file manipulation_pipeline/action.h
 * \brief Executable actions that act as the interface between planning steps and eventual
 * execution
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-16
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_ACTION_H_INCLUDED
#define MANIPULATION_PIPELINE_ACTION_H_INCLUDED

#include <memory>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

namespace moveit::core {
class RobotState;
}
namespace moveit_cpp {
class MoveItCpp;
}

namespace manipulation_pipeline {

class GoalHandle;
class MarkerInterface;

/*! \brief Interface for a single executable action
 */
class Action
{
public:
  explicit Action(std::string name);
  virtual ~Action() = default;

  [[nodiscard]] const std::string& name() const;

  virtual void execute(moveit_cpp::MoveItCpp& moveit_cpp, rclcpp::Logger& log) const = 0;
  virtual void visualize(const std::shared_ptr<MarkerInterface>& marker_interface,
                         GoalHandle& goal_handle) const;

private:
  std::string m_name;
};

/*! \brief Sequence of actions that should be executed sequentially
 *
 * This is the primary interface between the offline pipelined planning and the eventual execution
 * of actions.
 */
class ActionSequence
{
public:
  ActionSequence(std::string name, std::shared_ptr<GoalHandle> goal_handle);
  ActionSequence(std::string name,
                 std::shared_ptr<Action> first_action,
                 std::shared_ptr<GoalHandle> goal_handle);

  [[nodiscard]] const std::string& name() const;

  [[nodiscard]] const std::shared_ptr<GoalHandle>& goalHandle() const;

  void add(std::shared_ptr<Action> action);

  void execute(moveit_cpp::MoveItCpp& moveit_cpp, rclcpp::Logger log);
  void visualize(const std::shared_ptr<MarkerInterface>& marker_interface) const;

private:
  std::string m_name;
  std::shared_ptr<GoalHandle> m_goal_handle;

  std::vector<std::shared_ptr<Action>> m_actions;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_ACTION_H_INCLUDED
