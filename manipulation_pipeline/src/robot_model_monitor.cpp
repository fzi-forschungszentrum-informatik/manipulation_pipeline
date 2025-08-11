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
#include "manipulation_pipeline/robot_model_monitor.h"

namespace manipulation_pipeline {

RobotModelMonitor::RobotModelMonitor(std::function<void()> callback,
                                     const rclcpp::Node::SharedPtr& node,
                                     rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_node{node}
  , m_callback{std::move(callback)}
  , m_robot_description_sub{node->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS{1}.transient_local().reliable(),
      [this](const std_msgs::msg::String::ConstSharedPtr& msg) {
        descriptionCb("robot_description", msg, m_last_robot_description);
      })}
  , m_robot_description_semantic_sub{node->create_subscription<std_msgs::msg::String>(
      "robot_description_semantic",
      rclcpp::QoS{1}.transient_local().reliable(),
      [this](const std_msgs::msg::String::ConstSharedPtr& msg) {
        descriptionCb("robot_description_semantic", msg, m_last_robot_description_semantic);
      })}
{
}

void RobotModelMonitor::descriptionCb(const std::string& topic,
                                      const std_msgs::msg::String::ConstSharedPtr& msg,
                                      std_msgs::msg::String::ConstSharedPtr& last_msg)
{
  if (!msg)
  {
    return;
  }

  if (last_msg)
  {
    if (msg->data != last_msg->data)
    {
      RCLCPP_INFO(m_log, "Received new message on topic %s", topic.c_str());

      // Make sure to wait for other part of robot_description/robot_description_semantic
      m_wait_timer = m_node->create_wall_timer(std::chrono::seconds{1}, [this] {
        m_wait_timer->cancel();

        m_callback();
      });
    }
  }
  last_msg = msg;
}

} // namespace manipulation_pipeline
