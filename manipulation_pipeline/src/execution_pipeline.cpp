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
 * \date    2025-04-09
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/execution_pipeline.h"

#include "manipulation_pipeline/action.h"
#include "manipulation_pipeline/planning_step.h"

#include <manipulation_pipeline_interfaces/msg/progress.hpp>

namespace manipulation_pipeline {

ExecutionPipeline::ExecutionPipeline(moveit_cpp::MoveItCppPtr moveit_cpp, rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_moveit_cpp{std::move(moveit_cpp)}
  , m_worker{std::bind(&ExecutionPipeline::run, this, std::placeholders::_1, std::placeholders::_2)}
{
}

void ExecutionPipeline::execute(std::shared_ptr<ActionSequence> actions)
{
  actions->goalHandle()->feedback(
    manipulation_pipeline_interfaces::msg::Progress::PROGRESS_EXECUTION_QUEUED,
    fmt::format("Queued execution of action sequence '{}'", actions->name()));
  m_worker.push(std::move(actions));
}

void ExecutionPipeline::run(const std::shared_ptr<ActionSequence>& step, std::stop_token stop_token)
{
  try
  {
    step->execute(*m_moveit_cpp, m_log);
    step->goalHandle()->success("Successfully executed trajectory");
  }
  catch (std::runtime_error& ex)
  {
    RCLCPP_ERROR(m_log, "%s", ex.what());
    step->goalHandle()->failure(ex.what());
  }
}

} // namespace manipulation_pipeline
