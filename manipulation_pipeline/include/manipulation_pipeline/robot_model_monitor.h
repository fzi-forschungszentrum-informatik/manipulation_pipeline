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

/*!\file manipulation_pipeline/robot_model_monitor.h
 * \brief Monitor for notifying on robot description changes
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-02
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_ROBOT_MODEL_MONITOR_H_INCLUDED
#define MANIPULATION_PIPELINE_ROBOT_MODEL_MONITOR_H_INCLUDED

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace manipulation_pipeline {

/*! \brief Monitors robot_description and robot_description_semantic and notifies if it changes
 */
class RobotModelMonitor
{
public:
  RobotModelMonitor(std::function<void()> callback,
                    const rclcpp::Node::SharedPtr& node,
                    rclcpp::Logger log);

private:
  void descriptionCb(const std::string& topic,
                     const std_msgs::msg::String::ConstSharedPtr& msg,
                     std_msgs::msg::String::ConstSharedPtr& last_msg);

  rclcpp::Logger m_log;
  rclcpp::Node::SharedPtr m_node;

  std::function<void()> m_callback;

  rclcpp::TimerBase::SharedPtr m_wait_timer;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_description_sub;
  std_msgs::msg::String::ConstSharedPtr m_last_robot_description;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_description_semantic_sub;
  std_msgs::msg::String::ConstSharedPtr m_last_robot_description_semantic;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_ROBOT_MODEL_MONITOR_H_INCLUDED
