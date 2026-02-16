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
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

CollisionObjectManager::CollisionObjectManager(
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
  const rclcpp::Node::SharedPtr& node,
  rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_planning_scene_monitor{std::move(planning_scene_monitor)}
  , m_spawn_object_service{node->create_service<SpawnObjectSrv>(
      "~/spawn_object",
      std::bind(&CollisionObjectManager::spawnObjectCb,
                this,
                std::placeholders::_1,
                std::placeholders::_2))}
  , m_split_object_service{
      node->create_service<SplitObjectSrv>("~/split_object",
                                           std::bind(&CollisionObjectManager::splitObjectCb,
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

    if (request->attach_link.empty())
    {
      if (!m_planning_scene_monitor->processCollisionObjectMsg(collision_object, color))
      {
        throw std::runtime_error{"Could not process collision object"};
      }
    }
    else
    {
      const auto attached_collision_object =
        std::make_shared<moveit_msgs::msg::AttachedCollisionObject>();
      attached_collision_object->link_name = request->attach_link;
      attached_collision_object->object    = *collision_object;
      if (!m_planning_scene_monitor->processAttachedCollisionObjectMsg(attached_collision_object))
      {
        throw std::runtime_error{"Could not process attached collision object"};
      }
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

void CollisionObjectManager::splitObjectCb(std::shared_ptr<SplitObjectSrv::Request> request,
                                           std::shared_ptr<SplitObjectSrv::Response> response)
{
  try
  {
    // Get current object and check if it is attached anywhere
    const auto current_object = std::make_shared<moveit_msgs::msg::AttachedCollisionObject>();
    {
      planning_scene_monitor::LockedPlanningSceneRO planning_scene{m_planning_scene_monitor};
      if (!planning_scene->getAttachedCollisionObjectMsg(*current_object, request->name))
      {
        if (!planning_scene->getCollisionObjectMsg(current_object->object, request->name))
        {
          throw std::runtime_error{
            fmt::format("There is no collision object with name '{}'", request->name)};
        }
      }
    }

    const auto attach_str = current_object->link_name.empty()
                              ? "not attached"
                              : fmt::format("attached to {}", current_object->link_name);
    RCLCPP_INFO(m_log,
                "Found collision object %s at pose [%.05f, %.05f, %.05f](%.02f, %.02f, %.02f, "
                "%.02f) in frame %s (%s)",
                request->name.c_str(),
                current_object->object.pose.position.x,
                current_object->object.pose.position.y,
                current_object->object.pose.position.z,
                current_object->object.pose.orientation.x,
                current_object->object.pose.orientation.y,
                current_object->object.pose.orientation.z,
                current_object->object.pose.orientation.w,
                current_object->object.header.frame_id.c_str(),
                attach_str.c_str());

    response->success = true;
    response->message = fmt::format("Successfully split object '{}' into {} components",
                                    request->name,
                                    request->components.size());

    // Create new collision objects
    Eigen::Isometry3d current_pose;
    tf2::convert(current_object->object.pose, current_pose);

    std::vector<std::shared_ptr<moveit_msgs::msg::CollisionObject>> collision_objects;
    for (const auto& component : request->components)
    {
      Eigen::Isometry3d offset;
      tf2::convert(component.offset, offset);

      const auto component_object       = std::make_shared<moveit_msgs::msg::CollisionObject>();
      component_object->header.frame_id = current_object->object.header.frame_id;
      tf2::convert(current_pose * offset, component_object->pose);
      component_object->id = component.name;
      component_object->meshes.push_back(loadMesh(component.path));
      component_object->subframe_names = component.subframe_names;
      component_object->subframe_poses = component.subframe_poses;
      component_object->operation      = moveit_msgs::msg::CollisionObject::ADD;

      collision_objects.push_back(component_object);
    }

    // If object is currently attached, we first need to detach it
    if (!current_object->link_name.empty())
    {
      RCLCPP_INFO(m_log, "Detaching object %s", current_object->object.id.c_str());
      current_object->object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!m_planning_scene_monitor->processAttachedCollisionObjectMsg(current_object))
      {
        throw std::runtime_error{fmt::format("Unable to detach object {} from link {}",
                                             current_object->object.id,
                                             current_object->link_name)};
      }
    }

    // Remove current object
    RCLCPP_INFO(m_log, "Removing object %s from planning scene", current_object->object.id.c_str());
    const auto collision_object_buf = std::make_shared<moveit_msgs::msg::CollisionObject>();
    *collision_object_buf           = current_object->object;
    if (!m_planning_scene_monitor->processCollisionObjectMsg(collision_object_buf))
    {
      throw std::runtime_error{
        fmt::format("Unable to remove object {} from planning scene", collision_object_buf->id)};
    }

    // Spawn and attach new objects
    for (const auto& component_object : collision_objects)
    {
      RCLCPP_INFO(m_log, "Spawning collision object %s", component_object->id.c_str());
      if (!m_planning_scene_monitor->processCollisionObjectMsg(component_object))
      {
        throw std::runtime_error{
          fmt::format("Unable to spawn collision object for component {}", component_object->id)};
      }

      if (!current_object->link_name.empty())
      {
        RCLCPP_INFO(m_log,
                    "Attaching object %s to %s",
                    component_object->id.c_str(),
                    current_object->link_name.c_str());
        current_object->object = *component_object;
        if (!m_planning_scene_monitor->processAttachedCollisionObjectMsg(current_object))
        {
          throw std::runtime_error{fmt::format("Unable to attach new collision object {} to {}",
                                               current_object->object.id,
                                               current_object->link_name)};
        }
      }
    }
  }
  catch (const std::runtime_error& ex)
  {
    response->success = false;
    response->message = fmt::format("Could not split object '{}': {}", request->name, ex.what());
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
