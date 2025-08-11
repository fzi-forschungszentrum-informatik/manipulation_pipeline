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

/*!\file manipulation_pipeline/visualization_interface.h
 * \brief Interfaces for visualizing planning steps and their results
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-14
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_VISUALIZATION_INTERFACE_H_INCLUDED
#define MANIPULATION_PIPELINE_VISUALIZATION_INTERFACE_H_INCLUDED

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <span>
#include <std_msgs/msg/color_rgba.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace robot_trajectory {
class RobotTrajectory;
}
namespace moveit::core {
class LinkModel;
}

namespace manipulation_pipeline {

/*! \brief Interface for visualizations using a single marker publisher
 */
class MarkerInterface
{
public:
  MarkerInterface(const std::string& topic,
                  const rclcpp::Node::SharedPtr& node,
                  rclcpp::Logger log);

  std::size_t addFrame(const geometry_msgs::msg::PoseStamped& pose);
  std::size_t addFrame(const Eigen::Isometry3d& pose, const std::string& frame_id);

  std::size_t addLineString(const std::vector<geometry_msgs::msg::Point>& points,
                            const std::string& frame_id);
  std::size_t addPath(std::span<const geometry_msgs::msg::Pose> poses, const std::string& frame_id);
  std::size_t addPath(std::span<const Eigen::Isometry3d> poses, const std::string& frame_id);

  std::vector<std::size_t>
  addTrajectory(robot_trajectory::RobotTrajectory& trajectory,
                const std::vector<const moveit::core::LinkModel*>& tip_links);

  void publish();

  bool remove(std::size_t id);
  void clear();

private:
  using Marker = visualization_msgs::msg::Marker;

  rclcpp::Logger m_log;

  std_msgs::msg::ColorRGBA m_color_x;
  std_msgs::msg::ColorRGBA m_color_y;
  std_msgs::msg::ColorRGBA m_color_z;

  std::size_t m_id_cnt;
  std::unordered_map<std::size_t, Marker> m_markers;

  rclcpp::Publisher<Marker>::SharedPtr m_pub;
};

/*! \brief Interface to all visualization publishers used by the planning pipeline
 */
class VisualizationInterface
{
public:
  VisualizationInterface(const rclcpp::Node::SharedPtr& node, rclcpp::Logger log);

private:
  rclcpp::Logger m_log;

public:
  std::shared_ptr<MarkerInterface> plan_interface;
  std::shared_ptr<MarkerInterface> trajectory_interface;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_VISUALIZATION_INTERFACE_H_INCLUDED
