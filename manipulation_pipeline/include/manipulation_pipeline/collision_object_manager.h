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

/*!\file manipulation_pipeline/collision_object_manager.h
 * \brief Manage interactions with collision objects and provide ROS interfaces to them
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-02
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_COLLISION_OBJECT_MANAGER_H_INCLUDED
#define MANIPULATION_PIPELINE_COLLISION_OBJECT_MANAGER_H_INCLUDED

#include <manipulation_pipeline_interfaces/srv/combine_objects.hpp>
#include <manipulation_pipeline_interfaces/srv/spawn_object.hpp>
#include <manipulation_pipeline_interfaces/srv/split_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>

namespace manipulation_pipeline {

/*! \brief Provide interfaces to the collision objects defined in the planning scene.
 */
class CollisionObjectManager
{
public:
  CollisionObjectManager(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                         const rclcpp::Node::SharedPtr& node,
                         rclcpp::Logger log);

  std::vector<moveit_msgs::msg::CollisionObject> getCollisionObjects() const;
  void applyCollisionObjects(const std::vector<moveit_msgs::msg::CollisionObject>& objects) const;

private:
  using SpawnObjectSrv = manipulation_pipeline_interfaces::srv::SpawnObject;
  void spawnObjectCb(std::shared_ptr<SpawnObjectSrv::Request> request,
                     std::shared_ptr<SpawnObjectSrv::Response> response);

  using SplitObjectSrv = manipulation_pipeline_interfaces::srv::SplitObject;
  void splitObjectCb(std::shared_ptr<SplitObjectSrv::Request> request,
                     std::shared_ptr<SplitObjectSrv::Response> response);

  using CombineObjectsSrv = manipulation_pipeline_interfaces::srv::CombineObjects;
  void combineObjectsCb(std::shared_ptr<CombineObjectsSrv::Request> request,
                        std::shared_ptr<CombineObjectsSrv::Response> response);

  shape_msgs::msg::Mesh loadMesh(const std::string& path) const;

  rclcpp::Logger m_log;

  planning_scene_monitor::PlanningSceneMonitorPtr m_planning_scene_monitor;

  rclcpp::Service<SpawnObjectSrv>::SharedPtr m_spawn_object_service;
  rclcpp::Service<SplitObjectSrv>::SharedPtr m_split_object_service;
  rclcpp::Service<CombineObjectsSrv>::SharedPtr m_combine_object_service;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_COLLISION_OBJECT_MANAGER_H_INCLUDED
