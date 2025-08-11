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
#include "manipulation_pipeline/goal_handle.h"

namespace manipulation_pipeline {

GoalHandle::GoalHandle(std::size_t id)
  : m_id{id}
{
}

std::size_t GoalHandle::id() const
{
  return m_id;
}

void GoalHandle::addDoneCb(DoneCb cb)
{
  m_done_cbs.emplace_back(std::move(cb));
}

bool GoalHandle::isDone() const
{
  return m_done;
}

void GoalHandle::success(const std::string& message)
{
  signalSuccess(message);
  for (const auto& cb : m_done_cbs)
  {
    cb(true);
  }
  m_done = true;
}

void GoalHandle::failure(const std::string& message)
{
  signalFailure(message);
  for (const auto& cb : m_done_cbs)
  {
    cb(false);
  }
  m_done = true;
}

void GoalHandle::feedback(std::uint8_t progress,
                          const std::string& message,
                          std::uint8_t action_i,
                          std::uint8_t action_cnt)
{
  signalFeedback(progress, message, action_i, action_cnt);
}

} // namespace manipulation_pipeline
