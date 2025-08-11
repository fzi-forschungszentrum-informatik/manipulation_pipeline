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

/*!\file manipulation_pipeline/manipulation_pipeline.h
 * \brief Central manipulation pipeline node that orchestrates all other components
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-03-27
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_MANIPULATION_PIPELINE_H_INCLUDED
#define MANIPULATION_PIPELINE_MANIPULATION_PIPELINE_H_INCLUDED

#include "collision_object_manager.h"
#include "execution_pipeline.h"
#include "planning_interface.h"
#include "robot_model.h"
#include "robot_model_monitor.h"
#include "visualization_interface.h"

#include <manipulation_pipeline_interfaces/msg/state.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace manipulation_pipeline {

/*! \brief Central node that handles restarts and orchestrates all other components
 */
class ManipulationPipeline
{
public:
  explicit ManipulationPipeline(const rclcpp::Node::SharedPtr& node);

private:
  void reset();

  rclcpp::Node::SharedPtr m_node;

  using StateMsg = manipulation_pipeline_interfaces::msg::State;
  rclcpp::Publisher<StateMsg>::SharedPtr m_state_pub;
  StateMsg m_state;

  RobotModelMonitor m_robot_model_monitor;

  std::shared_ptr<VisualizationInterface> m_visualization_interface;

  std::shared_ptr<moveit_cpp::MoveItCpp> m_moveitcpp;
  std::shared_ptr<CollisionObjectManager> m_collision_object_manager;
  std::shared_ptr<PlanningInterface> m_planning_interface;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_MANIPULATION_PIPELINE_H_INCLUDED
