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

/*!\file manipulation_pipeline/robot_model.h
 * \brief Models and interfaces of joint groups that can be used to execute actions
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-07
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_ROBOT_MODEL_H_INCLUDED
#define MANIPULATION_PIPELINE_ROBOT_MODEL_H_INCLUDED

#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/logger.hpp>
#include <string>

namespace manipulation_pipeline {

/*! \brief Interface for planning or executing actions with a single joint group
 */
class GroupInterface
{
private:
  GroupInterface(const moveit::core::JointModelGroup* group,
                 const moveit::core::LinkModel* reference_link,
                 const std::vector<const moveit::core::LinkModel*> tip_links,
                 const GroupInterface* parent_group,
                 const moveit_cpp::MoveItCppPtr& moveit_cpp,
                 rclcpp::Logger log);

public:
  static GroupInterface createGroup(const moveit::core::JointModelGroup* group,
                                    const moveit_cpp::MoveItCppPtr& moveit_cpp,
                                    rclcpp::Logger log);
  static GroupInterface createChain(const moveit::core::JointModelGroup* group,
                                    const moveit_cpp::MoveItCppPtr& moveit_cpp,
                                    rclcpp::Logger log);
  static GroupInterface createEndEffector(const moveit::core::JointModelGroup* group,
                                          const GroupInterface* parent_group,
                                          const moveit_cpp::MoveItCppPtr& moveit_cpp,
                                          rclcpp::Logger log);

  [[nodiscard]] const std::string& name() const;
  [[nodiscard]] const moveit::core::JointModelGroup* group() const;
  [[nodiscard]] const GroupInterface& planningInterface() const;

  [[nodiscard]] const moveit::core::LinkModel* referenceLink() const;
  [[nodiscard]] const std::vector<const moveit::core::LinkModel*>& tipLinks() const;
  [[nodiscard]] const moveit::core::LinkModel* resolveTip(const std::string& tip_link) const;

  [[nodiscard]] moveit_cpp::PlanningComponent& planningComponent() const;

protected:
  rclcpp::Logger m_log;

  const moveit::core::JointModelGroup* m_group;
  const GroupInterface* m_parent_group;

  const moveit::core::LinkModel* m_reference_link;
  std::vector<const moveit::core::LinkModel*> m_tip_links;

  std::shared_ptr<moveit_cpp::PlanningComponent> m_planning_component;

private:
  static const moveit::core::LinkModel* getReferenceLink(const moveit::core::JointModelGroup* group,
                                                         rclcpp::Logger& log);
  static void getTipLinks(const srdf::Model::Group& group,
                          const moveit::core::RobotModel& robot_model,
                          std::vector<const moveit::core::LinkModel*>& tip_links,
                          rclcpp::Logger& log);
};

/*! \brief Model of all joint groups defined in the robot description
 */
class RobotModel
{
public:
  RobotModel(const moveit::core::RobotModel& model,
             const moveit_cpp::MoveItCppPtr& moveit_cpp,
             rclcpp::Logger log);

  /*! \brief Find a group interface for a given name
   *
   * This includes all kinds of groups, including end effectors. If name is empty and there is only
   * one group defined, this group will be used as a default.
   *
   * \param name Model name or empty string to signal default group
   */
  const GroupInterface& findModel(const std::string& name) const;

  /*! \brief Find a chain group for a given name
   *
   * If there is only one chain group defined in the SRDF, this group will be used as a default.
   *
   * \param name Chain group name or empty string to signal default
   */
  const GroupInterface& findChain(const std::string& name) const;

  /*! \brief Find an end effector group interface for a given name
   *
   * If name is empty and there is only one end effector group defined in the SRDF, this group
   * will be used as a default.
   *
   * \param name End effector name or empty string to signal default group
   */
  const GroupInterface& findEndEffector(const std::string& name) const;

  /*! \brief Get the model reference frame
   */
  const std::string& modelFrame() const;

private:
  const GroupInterface* findGroup(const std::vector<std::shared_ptr<GroupInterface>>& groups) const;

  rclcpp::Logger m_log;

  std::string m_model_frame;

  std::vector<std::shared_ptr<GroupInterface>> m_robot_interfaces;
  std::vector<std::shared_ptr<GroupInterface>> m_chains;
  std::vector<std::shared_ptr<GroupInterface>> m_end_effectors;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_ROBOT_MODEL_H_INCLUDED
