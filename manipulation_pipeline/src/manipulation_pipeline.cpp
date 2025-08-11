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
 * \date    2025-03-27
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/manipulation_pipeline.h"

#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace manipulation_pipeline {

ManipulationPipeline::ManipulationPipeline(const rclcpp::Node::SharedPtr& node)
  : m_node{node}
  , m_state_pub{m_node->create_publisher<StateMsg>("~/state",
                                                   rclcpp::QoS{1}.transient_local().reliable())}
  , m_robot_model_monitor{[this]() {
                            RCLCPP_INFO(m_node->get_logger(), "Resetting manipulation pipeline");
                            reset();
                          },
                          node,
                          node->get_logger().get_child("robot_model")}
  , m_visualization_interface{
      std::make_shared<VisualizationInterface>(node, node->get_logger().get_child("visualization"))}
{
  m_state.generation = 0;
  m_state_pub->publish(m_state);

  reset();
  RCLCPP_INFO(node->get_logger(), "Manipulation pipeline started");
}

void ManipulationPipeline::reset()
{
  // Preserve collision objects
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  if (m_collision_object_manager)
  {
    collision_objects = m_collision_object_manager->getCollisionObjects();
  }
  m_planning_interface.reset();
  m_moveitcpp.reset();

  // Clear parameters to ensure that the new descriptions are used
  if (m_node->has_parameter("robot_description"))
  {
    m_node->set_parameters({rclcpp::Parameter{"robot_description", ""}});
  }
  if (m_node->has_parameter("robot_description_semantic"))
  {
    m_node->set_parameters({rclcpp::Parameter{"robot_description_semantic", ""}});
  }

  // Set up pipeline
  rclcpp::Logger log = m_node->get_logger();

  m_moveitcpp = std::make_shared<moveit_cpp::MoveItCpp>(m_node);
  RCLCPP_INFO(m_node->get_logger(), "Planning pipelines:");
  for (const auto& [name, pipeline] : m_moveitcpp->getPlanningPipelines())
  {
    RCLCPP_INFO(m_node->get_logger(), "  - %s", name.c_str());
  }
  m_collision_object_manager = std::make_shared<CollisionObjectManager>(
    m_moveitcpp->getPlanningSceneMonitorNonConst(), m_node, log.get_child("collision_objects"));

  const auto robot_model =
    std::make_shared<RobotModel>(*m_moveitcpp->getPlanningSceneMonitor()->getRobotModel(),
                                 m_moveitcpp,
                                 log.get_child("robot_model"));
  const auto execution_pipeline =
    std::make_shared<ExecutionPipeline>(m_moveitcpp, log.get_child("execution_pipeline"));
  const auto planning_pipeline =
    std::make_shared<PlanningPipeline>(m_moveitcpp,
                                       robot_model,
                                       execution_pipeline,
                                       m_visualization_interface,
                                       m_node,
                                       log.get_child("planning_pipeline"));
  m_planning_interface = std::make_shared<PlanningInterface>(
    planning_pipeline, m_node, log.get_child("planning_interface"));

  // Re-apply collision objects
  m_collision_object_manager->applyCollisionObjects(collision_objects);

  m_moveitcpp->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

  // Update state message
  m_state.generation++;
  m_state_pub->publish(m_state);
}

} // namespace manipulation_pipeline


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create node
  rclcpp::NodeOptions node_options{};
  node_options.automatically_declare_parameters_from_overrides(true);
  const auto node = std::make_shared<rclcpp::Node>("manipulation_pipeline", "", node_options);

  // Spin executor
  rclcpp::executors::SingleThreadedExecutor executor{};
  executor.add_node(node);
  std::thread executor_thread{[&executor]() { executor.spin(); }};

  // Instantiate manipulation pipeline
  const manipulation_pipeline::ManipulationPipeline mapi{node};
  executor_thread.join();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
