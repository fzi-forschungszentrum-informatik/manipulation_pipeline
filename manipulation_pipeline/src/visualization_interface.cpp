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
 * \date    2025-04-14
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/visualization_interface.h"

#include <fmt/format.h>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

MarkerInterface::MarkerInterface(const std::string& topic,
                                 const rclcpp::Node::SharedPtr& node,
                                 rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_id_cnt{0}
  , m_pub{node->create_publisher<Marker>(topic, rclcpp::SystemDefaultsQoS{})}
{
  m_color_x.r = 1.0;
  m_color_x.a = 1.0;

  m_color_y.g = 1.0;
  m_color_y.a = 1.0;

  m_color_z.b = 1.0;
  m_color_z.a = 1.0;
}

std::size_t MarkerInterface::addFrame(const Eigen::Isometry3d& pose, const std::string& frame_id)
{
  constexpr double FRAME_SIZE = 0.25;

  Marker marker;
  marker.header.frame_id = frame_id;
  marker.id              = m_id_cnt++;
  marker.type            = Marker::LINE_LIST;
  marker.action          = Marker::ADD;

  tf2::convert(pose, marker.pose);
  marker.scale.x = 0.01;

  geometry_msgs::msg::Point point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;

  marker.points.push_back(point);
  marker.colors.push_back(m_color_x);
  point.x = FRAME_SIZE;
  marker.points.push_back(point);
  marker.colors.push_back(m_color_x);
  point.x = 0.0;

  marker.points.push_back(point);
  marker.colors.push_back(m_color_y);
  point.y = FRAME_SIZE;
  marker.points.push_back(point);
  marker.colors.push_back(m_color_y);
  point.y = 0.0;

  marker.points.push_back(point);
  marker.colors.push_back(m_color_z);
  point.z = FRAME_SIZE;
  marker.points.push_back(point);
  marker.colors.push_back(m_color_z);
  point.z = 0.0;

  m_markers.emplace(marker.id, marker);

  return marker.id;
}

std::size_t MarkerInterface::addFrame(const geometry_msgs::msg::PoseStamped& pose)
{
  Eigen::Isometry3d pose_eigen;
  tf2::convert(pose.pose, pose_eigen);
  return addFrame(pose_eigen, pose.header.frame_id);
}

std::size_t MarkerInterface::addLineString(const std::vector<geometry_msgs::msg::Point>& points,
                                           const std::string& frame_id)
{
  if (points.size() < 2)
  {
    RCLCPP_WARN(m_log, "Not publishing trajectory with only %zu points (min 2)", points.size());
    return m_id_cnt++;
  }

  Marker marker;
  marker.header.frame_id = frame_id;
  marker.id              = m_id_cnt++;
  marker.type            = Marker::LINE_STRIP;
  marker.action          = Marker::ADD;

  marker.scale.x = 0.01;

  marker.color.a = 1.0;
  marker.color.g = 1.0;

  for (const auto& point : points)
  {
    marker.points.push_back(point);
  }

  m_markers.emplace(marker.id, marker);

  return marker.id;
}

std::size_t MarkerInterface::addPath(std::span<const geometry_msgs::msg::Pose> poses,
                                     const std::string& frame_id)
{
  std::vector<Eigen::Isometry3d> poses_eigen;
  std::transform(
    poses.begin(), poses.end(), std::back_inserter(poses_eigen), [](const auto& pose_msg) {
      Eigen::Isometry3d pose;
      tf2::convert(pose_msg, pose);
      return pose;
    });

  return addPath(poses_eigen, frame_id);
}

std::size_t MarkerInterface::addPath(std::span<const Eigen::Isometry3d> poses,
                                     const std::string& frame_id)
{
  constexpr double FRAME_SIZE = 0.05;

  Marker marker;
  marker.header.frame_id = frame_id;
  marker.id              = m_id_cnt++;
  marker.type            = Marker::LINE_LIST;
  marker.action          = Marker::ADD;

  marker.scale.x = 0.001;

  const auto apply_offset = [](const Eigen::Isometry3d& pose, const Eigen::Vector3d& offset) {
    Eigen::Vector3d point_offset =
      (Eigen::Translation3d{pose.translation()} * Eigen::Translation3d{pose.linear() * offset})
        .translation();

    geometry_msgs::msg::Point result;
    tf2::convert(point_offset, result);

    return result;
  };

  for (const auto& pose : poses)
  {
    geometry_msgs::msg::Point point;
    tf2::convert(Eigen::Vector3d{pose.translation()}, point);

    const auto point_x = apply_offset(pose, FRAME_SIZE * Eigen::Vector3d::UnitX());
    const auto point_y = apply_offset(pose, FRAME_SIZE * Eigen::Vector3d::UnitY());
    const auto point_z = apply_offset(pose, FRAME_SIZE * Eigen::Vector3d::UnitZ());

    marker.points.push_back(point);
    marker.points.push_back(point_x);
    marker.colors.push_back(m_color_x);
    marker.colors.push_back(m_color_x);

    marker.points.push_back(point);
    marker.points.push_back(point_y);
    marker.colors.push_back(m_color_y);
    marker.colors.push_back(m_color_y);

    marker.points.push_back(point);
    marker.points.push_back(point_z);
    marker.colors.push_back(m_color_z);
    marker.colors.push_back(m_color_z);
  }

  std_msgs::msg::ColorRGBA path_color;
  path_color.g = 1.0;
  path_color.a = 1.0;
  for (std::size_t i = 1; i < poses.size(); ++i)
  {
    geometry_msgs::msg::Point point;
    tf2::convert(Eigen::Vector3d{poses[i - 1].translation()}, point);
    marker.points.push_back(point);
    tf2::convert(Eigen::Vector3d{poses[i].translation()}, point);
    marker.points.push_back(point);

    marker.colors.push_back(path_color);
    marker.colors.push_back(path_color);
  }

  m_markers.emplace(marker.id, marker);
  return marker.id;
}

std::vector<std::size_t>
MarkerInterface::addTrajectory(robot_trajectory::RobotTrajectory& trajectory,
                               const std::vector<const moveit::core::LinkModel*>& tip_links)
{
  std::vector<std::size_t> marker_ids;

  const auto& robot_model = trajectory.getRobotModel();

  const auto& frame = robot_model->getModelFrame();
  std::vector<std::vector<geometry_msgs::msg::Pose>> poses;
  poses.resize(tip_links.size());

  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i)
  {
    const auto robot_state = trajectory.getWayPoint(i);

    for (std::size_t tip_i = 0; tip_i < tip_links.size(); ++tip_i)
    {
      const auto* tip = tip_links[tip_i];
      if (!robot_state.knowsFrameTransform(tip->getName()))
      {
        throw std::runtime_error{
          fmt::format("Could not find tip transform for tip '{}'", tip->getName())};
      }

      const auto& tip_pose = robot_state.getGlobalLinkTransform(tip);

      geometry_msgs::msg::Pose pose;
      tf2::convert(tip_pose, pose);
      poses[tip_i].push_back(pose);
    }
  }

  for (const auto& tip_poses : poses)
  {
    std::span<const geometry_msgs::msg::Pose> pose_span{tip_poses.cbegin(), tip_poses.cend()};
    marker_ids.push_back(addPath(pose_span, frame));
  }

  return marker_ids;
}

void MarkerInterface::publish()
{
  for (const auto& [id, marker] : m_markers)
  {
    m_pub->publish(marker);
  }
}

bool MarkerInterface::remove(std::size_t id)
{
  if (const auto& marker_it = m_markers.find(id); marker_it != m_markers.end())
  {
    Marker remove_marker;
    remove_marker.header.frame_id = marker_it->second.header.frame_id;
    remove_marker.action          = Marker::DELETE;
    remove_marker.id              = id;

    m_pub->publish(remove_marker);
    m_markers.erase(marker_it);

    return true;
  }
  else
  {
    return false;
  }
}

void MarkerInterface::clear()
{
  if (!m_markers.empty())
  {
    Marker marker;
    marker.header.frame_id =
      m_markers.begin()->second.header.frame_id; // rviz ignores message without frame_id
    marker.action = Marker::DELETEALL;

    m_pub->publish(marker);
  }

  m_markers.clear();
}

VisualizationInterface::VisualizationInterface(const rclcpp::Node::SharedPtr& node,
                                               rclcpp::Logger log)
  : m_log{std::move(log)}
  , plan_interface{std::make_shared<MarkerInterface>("~/plan", node, m_log)}
  , trajectory_interface{std::make_shared<MarkerInterface>("~/trajectory", node, m_log)}
{
}

} // namespace manipulation_pipeline
