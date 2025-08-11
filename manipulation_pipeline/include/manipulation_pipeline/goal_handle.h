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

/*!\file manipulation_pipeline/goal_handle.h
 * \brief Handles for currently running actions
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-16
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_GOAL_HANDLE_H_INCLUDED
#define MANIPULATION_PIPELINE_GOAL_HANDLE_H_INCLUDED

#include <functional>
#include <rclcpp_action/server_goal_handle.hpp>
#include <string>
#include <vector>

namespace manipulation_pipeline {

/*! \brief Base handle for currently running actions
 */
class GoalHandle
{
public:
  explicit GoalHandle(std::size_t id);

  [[nodiscard]] std::size_t id() const;

  using DoneCb = std::function<void(bool)>;
  void addDoneCb(DoneCb cb);

  [[nodiscard]] bool isDone() const;

  void success(const std::string& message);
  void failure(const std::string& message);
  void feedback(std::uint8_t progress,
                const std::string& message,
                std::uint8_t action_i   = 0,
                std::uint8_t action_cnt = 0);

protected:
  virtual void signalSuccess(const std::string& message) const = 0;
  virtual void signalFailure(const std::string& message) const = 0;
  virtual void signalFeedback(std::uint8_t progress,
                              const std::string& message,
                              std::uint8_t action_i,
                              std::uint8_t action_cnt) const   = 0;

private:
  std::size_t m_id;

  bool m_done = false;
  std::vector<DoneCb> m_done_cbs;
};

/*! \brief GoalHandle specialization for specific action types
 *
 * \tparam ActionT Action type this handle references
 */
template <typename ActionT>
class ActionGoalHandle : public GoalHandle
{
public:
  explicit ActionGoalHandle(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle,
                            std::size_t id);

  const rclcpp_action::ServerGoalHandle<ActionT>& handle() const;

protected:
  void signalSuccess(const std::string& message) const override;
  void signalFailure(const std::string& message) const override;
  void signalFeedback(std::uint8_t progress,
                      const std::string& message,
                      std::uint8_t action_i,
                      std::uint8_t action_cnt) const override;

private:
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> m_handle;
};

} // namespace manipulation_pipeline


namespace manipulation_pipeline {

template <typename ActionT>
ActionGoalHandle<ActionT>::ActionGoalHandle(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle, std::size_t id)
  : GoalHandle{id}
  , m_handle{std::move(handle)}
{
}

template <typename ActionT>
const rclcpp_action::ServerGoalHandle<ActionT>& ActionGoalHandle<ActionT>::handle() const
{
  return *m_handle;
}

template <typename ActionT>
void ActionGoalHandle<ActionT>::signalSuccess(const std::string& message) const
{
  const auto result = std::make_shared<typename ActionT::Result>();
  result->success   = true;
  result->message   = message;
  m_handle->succeed(result);
}

template <typename ActionT>
void ActionGoalHandle<ActionT>::signalFailure(const std::string& message) const
{
  const auto result = std::make_shared<typename ActionT::Result>();
  result->success   = false;
  result->message   = message;
  m_handle->abort(result);
}

template <typename ActionT>
void ActionGoalHandle<ActionT>::signalFeedback(std::uint8_t progress,
                                               const std::string& message,
                                               std::uint8_t action_i,
                                               std::uint8_t action_cnt) const
{
  const auto feedback         = std::make_shared<typename ActionT::Feedback>();
  feedback->progress.progress = progress;
  feedback->progress.message  = message;

  feedback->progress.execution_action_count = action_cnt;
  feedback->progress.execution_action_index = action_i;

  m_handle->publish_feedback(feedback);
}

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_GOAL_HANDLE_H_INCLUDED
