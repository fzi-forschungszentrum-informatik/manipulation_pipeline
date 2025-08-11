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

/*!\file manipulation_pipeline/planning_context.h
 * \brief Context for planning single action planning steps
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-15
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_PLANNING_CONTEXT_H_INCLUDED
#define MANIPULATION_PIPELINE_PLANNING_CONTEXT_H_INCLUDED

#include <memory>
#include <unordered_map>

namespace rclcpp {
class Node;
}
namespace planning_pipeline {
class PlanningPipeline;
}
namespace planning_scene {
class PlanningScene;
}

namespace manipulation_pipeline {

class MarkerInterface;

/*! \brief Planning context provided for action planning steps
 */
struct PlanningContext
{
  std::shared_ptr<rclcpp::Node> node;

  std::shared_ptr<planning_scene::PlanningScene> planning_scene;
  std::unordered_map<std::string, std::shared_ptr<planning_pipeline::PlanningPipeline>>
    planning_pipelines;

  MarkerInterface* plan_visualizer;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_PLANNING_CONTEXT_H_INCLUDED
