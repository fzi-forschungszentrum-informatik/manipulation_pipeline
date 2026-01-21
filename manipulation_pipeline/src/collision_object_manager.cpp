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
 * \date    2025-04-02
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/collision_object_manager.h"

#include <fmt/format.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>

namespace manipulation_pipeline {

CollisionObjectManager::CollisionObjectManager(
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
  const rclcpp::Node::SharedPtr& node,
  rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_planning_scene_monitor{std::move(planning_scene_monitor)}
  , m_spawn_object_service{
      node->create_service<SpawnObjectSrv>("~/spawn_object",
                                           std::bind(&CollisionObjectManager::spawnObjectCb,
                                                     this,
                                                     std::placeholders::_1,
                                                     std::placeholders::_2))}
{
}

std::vector<moveit_msgs::msg::CollisionObject> CollisionObjectManager::getCollisionObjects() const
{
  std::vector<moveit_msgs::msg::CollisionObject> result;

  planning_scene_monitor::LockedPlanningSceneRO planning_scene{m_planning_scene_monitor};
  planning_scene->getCollisionObjectMsgs(result);
  return result;
}

void CollisionObjectManager::applyCollisionObjects(
  const std::vector<moveit_msgs::msg::CollisionObject>& objects) const
{
  const auto ptr = std::make_shared<moveit_msgs::msg::CollisionObject>();
  for (const auto& object : objects)
  {
    *ptr = object;
    if (!m_planning_scene_monitor->processCollisionObjectMsg(ptr))
    {
      RCLCPP_ERROR(m_log, "Could not apply collision object '%s'", object.id.c_str());
    }
  }
}

void CollisionObjectManager::spawnObjectCb(std::shared_ptr<SpawnObjectSrv::Request> request,
                                           std::shared_ptr<SpawnObjectSrv::Response> response)
{
  try
  {
    // Check if the object already exists
    // In the normal case, this will just move the object, but if it is currently attached, we need
    // to remove it first
    {
      moveit_msgs::msg::AttachedCollisionObject attached_co;

      planning_scene_monitor::LockedPlanningSceneRW planning_scene{m_planning_scene_monitor};
      if (planning_scene->getAttachedCollisionObjectMsg(attached_co, request->name))
      {
        RCLCPP_INFO(m_log,
                    "Object %s already exists and is attached to %s, detaching first",
                    attached_co.object.id.c_str(),
                    attached_co.link_name.c_str());
        attached_co.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene->processAttachedCollisionObjectMsg(attached_co);

        m_planning_scene_monitor->triggerSceneUpdateEvent(
          planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_GEOMETRY);
      }
    }

    const auto collision_object       = std::make_shared<moveit_msgs::msg::CollisionObject>();
    collision_object->header.frame_id = request->pose.header.frame_id;
    collision_object->pose            = request->pose.pose;
    collision_object->id              = request->name;
    collision_object->meshes.push_back(loadMesh(request->path));
    collision_object->subframe_names = request->subframe_names;
    collision_object->subframe_poses = request->subframe_poses;
    collision_object->operation      = moveit_msgs::msg::CollisionObject::ADD;

    std::optional<moveit_msgs::msg::ObjectColor> color = std::nullopt;
    if (request->color.a != 0.0)
    {
      color.emplace();
      color->id    = request->name;
      color->color = request->color;
    }

    if (!m_planning_scene_monitor->processCollisionObjectMsg(collision_object, color))
    {
      throw std::runtime_error{"Could not process collision object"};
    }

    response->success = true;
    response->message = fmt::format("Successfully spawned object '{}'", request->name);
    RCLCPP_INFO(m_log, "%s", response->message.c_str());
  }
  catch (const std::runtime_error& ex)
  {
    response->success = false;
    response->message = fmt::format("Could not spawn object '{}': {}", request->name, ex.what());
    RCLCPP_ERROR(m_log, "%s", response->message.c_str());
  }
}

shape_msgs::msg::Mesh CollisionObjectManager::loadMesh(const std::string& path) const
{
  const std::unique_ptr<shapes::Mesh> mesh{shapes::createMeshFromResource(path)};
  if (!mesh)
  {
    throw std::runtime_error{"Could not load mesh"};
  }
  RCLCPP_INFO(m_log, "Successfully loaded mesh from path '%s'", path.c_str());

  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh.get(), shape_msg);

  return boost::get<shape_msgs::msg::Mesh>(shape_msg);
}

} // namespace manipulation_pipeline
