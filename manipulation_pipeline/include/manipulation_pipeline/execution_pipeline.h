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

/*!\file manipulation_pipeline/execution_pipeline.h
 * \brief Sequential executor for previously planned actions
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-09
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_EXECUTION_PIPELINE_H_INCLUDED
#define MANIPULATION_PIPELINE_EXECUTION_PIPELINE_H_INCLUDED

#include "worker_thread.h"

#include <rclcpp/logger.hpp>

namespace moveit_cpp {
class MoveItCpp;
}

namespace manipulation_pipeline {

class ActionSequence;

/*! \brief Sequential executor for previously planned actions
 */
class ExecutionPipeline
{
public:
  ExecutionPipeline(std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp, rclcpp::Logger log);

  void execute(std::shared_ptr<ActionSequence> action_sequence);

private:
  void run(const std::shared_ptr<ActionSequence>& actions, std::stop_token stop_token);

  rclcpp::Logger m_log;

  std::shared_ptr<moveit_cpp::MoveItCpp> m_moveit_cpp;
  WorkerThread<std::shared_ptr<ActionSequence>> m_worker;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_EXECUTION_PIPELINE_H_INCLUDED
