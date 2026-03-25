// Copyright 2026 FZI Forschungszentrum Informatik
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

/*!\file manipulation_pipeline/planning_steps/manipulation_planning_step.h
 * \brief Base for manipulation steps that interact with collision objects
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2026-03-25
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_MANIPULATION_PLANNING_STEPS_H_INCLUDED
#define MANIPULATION_PIPELINE_MANIPULATION_PLANNING_STEPS_H_INCLUDED

#include "manipulation_pipeline/planning_step.h"

#include <Eigen/Geometry>

namespace planning_scene {
class PlanningScene;
}
namespace moveit::core {
class LinkModel;
class JointModelGroup;
} // namespace moveit::core

namespace manipulation_pipeline {

class ManipulationPlanningStepBase
{
public:
  virtual ~ManipulationPlanningStepBase() = default;

protected:
};

template <typename ActionT>
class ManipulationPlanningStep
  : public ActionPlanningStep<ActionT>
  , public ManipulationPlanningStepBase
{
public:
  ManipulationPlanningStep(const std::shared_ptr<ActionGoalHandle<ActionT>>& goal_handle,
                           std::string name,
                           rclcpp::Logger log);
};

} // namespace manipulation_pipeline


namespace manipulation_pipeline {

template <typename ActionT>
ManipulationPlanningStep<ActionT>::ManipulationPlanningStep(
  const std::shared_ptr<ActionGoalHandle<ActionT>>& goal_handle,
  std::string name,
  rclcpp::Logger log)
  : ActionPlanningStep<ActionT>{goal_handle, std::move(name), std::move(log)}
{
}

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_MANIPULATION_PLANNING_STEPS_H_INCLUDE:D
