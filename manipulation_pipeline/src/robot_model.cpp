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
 * \date    2025-04-07
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/robot_model.h"

#include <fmt/format.h>
#include <rclcpp/logging.hpp>

namespace manipulation_pipeline {

GroupInterface::GroupInterface(const moveit::core::JointModelGroup* group,
                               const moveit::core::LinkModel* reference_link,
                               const std::vector<const moveit::core::LinkModel*> tip_links,
                               const GroupInterface* parent_group,
                               const moveit_cpp::MoveItCppPtr& moveit_cpp,
                               rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_group{group}
  , m_parent_group{parent_group}
  , m_reference_link{reference_link}
  , m_tip_links{tip_links}
  , m_planning_component{
      std::make_shared<moveit_cpp::PlanningComponent>(group->getName(), moveit_cpp)}
{
}

GroupInterface GroupInterface::createGroup(const moveit::core::JointModelGroup* group,
                                           const moveit_cpp::MoveItCppPtr& moveit_cpp,
                                           rclcpp::Logger log)
{
  const auto* reference_link = getReferenceLink(group, log);

  std::vector<const moveit::core::LinkModel*> tip_links;
  getTipLinks(group->getConfig(), group->getParentModel(), tip_links, log);

  return GroupInterface{group, reference_link, tip_links, nullptr, moveit_cpp, std::move(log)};
}

GroupInterface GroupInterface::createChain(const moveit::core::JointModelGroup* group,
                                           const moveit_cpp::MoveItCppPtr& moveit_cpp,
                                           rclcpp::Logger log)
{
  const auto& chains = group->getConfig().chains_;
  if (chains.size() != 1)
  {
    throw std::runtime_error{
      fmt::format("Expected chain group '{}' to be defined as exactly one chain, but got {}",
                  group->getName().c_str(),
                  chains.size())};
  }
  const auto& chain = chains[0];

  // Base link is not part of joint model group
  const auto* base_link = group->getParentModel().getLinkModel(chain.first);
  if (!base_link)
  {
    throw std::runtime_error{
      fmt::format("Invalid chain base link '{}' for group '{}'", chain.first, group->getName())};
  }
  const auto* tip_link = group->getLinkModel(chain.second);
  if (!tip_link)
  {
    throw std::runtime_error{
      fmt::format("Invalid chain tip link '{}' for group '{}'", chain.second, group->getName())};
  }

  return GroupInterface{group, base_link, {tip_link}, nullptr, moveit_cpp, std::move(log)};
}

GroupInterface GroupInterface::createEndEffector(const moveit::core::JointModelGroup* group,
                                                 const GroupInterface* parent_group,
                                                 const moveit_cpp::MoveItCppPtr& moveit_cpp,
                                                 rclcpp::Logger log)
{
  const auto* reference_link = getReferenceLink(group, log);

  return GroupInterface{group, reference_link, {}, parent_group, moveit_cpp, std::move(log)};
}

const std::string& GroupInterface::name() const
{
  return m_group->getName();
}

const moveit::core::JointModelGroup* GroupInterface::group() const
{
  return m_group;
}

const GroupInterface& GroupInterface::planningInterface() const
{
  if (m_parent_group)
  {
    return *m_parent_group;
  }

  return *this;
}

const moveit::core::LinkModel* GroupInterface::referenceLink() const
{
  return m_reference_link;
}

const std::vector<const moveit::core::LinkModel*>& GroupInterface::tipLinks() const
{
  return m_tip_links;
}

const moveit::core::LinkModel* GroupInterface::resolveTip(const std::string& tip_link) const
{
  if (m_tip_links.empty())
  {
    throw std::runtime_error{
      fmt::format("Cannot resolve tips link for robot model '{}' as it has no tip links",
                  m_group->getParentModel().getName())};
  }

  const auto& robot_model = m_group->getParentModel();
  if (tip_link.empty())
  {
    if (m_tip_links.size() == 1)
    {
      return m_tip_links[0];
    }
    else
    {
      throw std::runtime_error{fmt::format(
        "Cannot automatically deduce tip link as there are {} defined tips for robot model '{}'",
        m_tip_links.size(),
        robot_model.getName())};
    }
  }

  const auto* link_model = robot_model.getLinkModel(tip_link);
  if (!link_model)
  {
    throw std::runtime_error{fmt::format("Tip link '{}' unknown", tip_link)};
  }

  return link_model;
}

moveit_cpp::PlanningComponent& GroupInterface::planningComponent() const
{
  return *m_planning_component;
}

const moveit::core::LinkModel*
GroupInterface::getReferenceLink(const moveit::core::JointModelGroup* group, rclcpp::Logger& log)
{
  // Try getting common root joint
  if (const auto* group_root = group->getCommonRoot(); group_root)
  {
    if (const auto* parent_link = group_root->getParentLinkModel(); parent_link)
    {
      return parent_link;
    }
    else
    {
      throw std::runtime_error{
        fmt::format("Group root joint '{}' of group '{}' doesn't have a parent link",
                    group_root->getName(),
                    group->getName())};
    }
  }

  // If this is not possible, use model root
  if (const auto* model_root = group->getParentModel().getRootLink(); model_root)
  {
    return model_root;
  }
  else
  {
    throw std::runtime_error{fmt::format("Could not get model root for robot model '{}'",
                                         group->getParentModel().getName())};
  }
}

void GroupInterface::getTipLinks(const srdf::Model::Group& group,
                                 const moveit::core::RobotModel& robot_model,
                                 std::vector<const moveit::core::LinkModel*>& tip_links,
                                 rclcpp::Logger& log)
{
  if (!group.joints_.empty())
  {
    RCLCPP_WARN(log,
                "Ignoring %zu directly specified joints for tip determination in group %s",
                group.joints_.size(),
                group.name_.c_str());
  }
  if (!group.links_.empty())
  {
    RCLCPP_WARN(log,
                "Ignoring %zu directly specified links for tip determination in group %s",
                group.links_.size(),
                group.name_.c_str());
  }

  for (const auto& chain : group.chains_)
  {
    const auto* chain_tip_link = robot_model.getLinkModel(chain.second);
    if (chain_tip_link)
    {
      tip_links.push_back(chain_tip_link);
    }
    else
    {
      RCLCPP_WARN(log,
                  "Could not find link model for chain tip '%s' of group '%s'",
                  chain.second.c_str(),
                  group.name_.c_str());
    }
  }

  const auto& srdf_model = robot_model.getSRDF();
  for (const auto& group_name : group.subgroups_)
  {
    const auto& groups = srdf_model->getGroups();
    if (const auto group_it =
          std::find_if(groups.begin(),
                       groups.end(),
                       [&](const auto& subgroup) { return subgroup.name_ == group_name; });
        group_it != groups.end())
    {
      getTipLinks(*group_it, robot_model, tip_links, log);
    }
    else
    {
      RCLCPP_WARN(log,
                  "Could not find subgroup '%s' of group '%s' in SRDF",
                  group_name.c_str(),
                  group.name_.c_str());
    }
  }
}

RobotModel::RobotModel(const moveit::core::RobotModel& model,
                       const moveit_cpp::MoveItCppPtr& moveit_cpp,
                       rclcpp::Logger log)
  : m_log{std::move(log)}
{
  RCLCPP_INFO(m_log, "Parsing robot model '%s'", model.getName().c_str());

  const auto print_interface = [&](const auto& interface, const std::string& type) {
    // Print interface information
    const auto& tip_links = interface->tipLinks();
    std::vector<std::string> tip_names;
    std::transform(tip_links.begin(),
                   tip_links.end(),
                   std::back_inserter(tip_names),
                   [](const auto* tip_link) { return tip_link->getName(); });
    const auto tip_str = fmt::format("[{}]", fmt::join(tip_names, ","));

    RCLCPP_INFO(m_log,
                "  - %s [%s](reference=%s, tips=%s)",
                interface->name().c_str(),
                type.c_str(),
                interface->referenceLink()->getName().c_str(),
                tip_str.c_str());
  };

  for (const auto* group : model.getJointModelGroups())
  {
    if (group->isEndEffector())
    {
      // Register these in the second pass
      continue;
    }

    const auto group_log = m_log.get_child(group->getName());
    if (group->isChain())
    {
      const auto interface =
        std::make_shared<GroupInterface>(GroupInterface::createChain(group, moveit_cpp, group_log));
      print_interface(interface, "Chain");

      m_robot_interfaces.push_back(interface);
      m_chains.push_back(interface);
    }
    else
    {
      const auto interface =
        std::make_shared<GroupInterface>(GroupInterface::createGroup(group, moveit_cpp, group_log));
      print_interface(interface, "Group");

      m_robot_interfaces.push_back(interface);
    }
  }

  // Do a second pass so we can link end-effectors to the interfaces belonging to their parent
  // groups
  for (const auto* group : model.getJointModelGroups())
  {
    if (group->isEndEffector())
    {
      const auto group_log = m_log.get_child(group->getName());

      // Find interface belonging to parent group
      const auto parent_group_name = group->getEndEffectorParentGroup().first;
      const auto* parent_interface = [&] {
        if (const auto find_it =
              std::find_if(m_robot_interfaces.begin(),
                           m_robot_interfaces.end(),
                           [&](const auto& group) { return group->name() == parent_group_name; });
            find_it != m_robot_interfaces.end())
        {
          return (*find_it).get();
        }
        else if (const auto find_it = std::find_if(
                   m_chains.begin(),
                   m_chains.end(),
                   [&](const auto& chain) { return chain->name() == parent_group_name; });
                 find_it != m_chains.end())
        {
          return (*find_it).get();
        }

        throw std::runtime_error{fmt::format(
          "Could not find parent interface for end effector parent group '{}'", parent_group_name)};
      }();

      const auto interface = std::make_shared<GroupInterface>(
        GroupInterface::createEndEffector(group, parent_interface, moveit_cpp, group_log));
      print_interface(interface, "EE");

      m_end_effectors.push_back(interface);
    }
  }
}

const GroupInterface& RobotModel::findModel(const std::string& name) const
{
  // Search by name
  if (!name.empty())
  {
    if (const auto find_it = std::find_if(
          m_robot_interfaces.begin(),
          m_robot_interfaces.end(),
          [&](const auto& robot_interface) { return robot_interface->name() == name; });
        find_it != m_robot_interfaces.end())
    {
      return *(*find_it);
    }
    else if (const auto find_it = std::find_if(m_end_effectors.begin(),
                                               m_end_effectors.end(),
                                               [&](const auto& ee) { return ee->name() == name; });
             find_it != m_end_effectors.end())
    {
      return *(*find_it);
    }

    throw std::runtime_error{fmt::format("There is no group named '{}'", name)};
  }

  const auto num_groups = m_robot_interfaces.size() + m_end_effectors.size();
  if (num_groups == 1)
  {
    const auto* default_model = [&] {
      if (m_robot_interfaces.size() == 1)
      {
        return m_robot_interfaces[0].get();
      }
      else
      {
        return m_end_effectors[0].get();
      }
    }();

    RCLCPP_INFO(m_log, "Defaulting to robot model '%s'", default_model->name().c_str());
    return *default_model;
  }
  else
  {
    throw std::runtime_error{
      "Cannot choose unique robot interfaces, as there are multiple possible groups defined"};
  }
}

const GroupInterface& RobotModel::findChain(const std::string& name) const
{
  // Search by name
  if (!name.empty())
  {
    if (const auto find_it = std::find_if(m_chains.begin(),
                                          m_chains.end(),
                                          [&](const auto& chain) { return chain->name() == name; });
        find_it != m_chains.end())
    {
      return *(*find_it);
    }

    throw std::runtime_error{fmt::format("There is no chain group named '{}'", name)};
  }

  if (m_chains.size() == 1)
  {
    const auto& default_group = *m_chains[0];
    RCLCPP_INFO(m_log, "Defaulting to chain group '%s'", default_group.name().c_str());
    return default_group;
  }
  else
  {
    throw std::runtime_error{
      "Cannot choose default chain group as there are multiple possible groups defined"};
  }
}

const GroupInterface& RobotModel::findEndEffector(const std::string& name) const
{
  // Search by name
  if (!name.empty())
  {
    if (const auto find_it = std::find_if(m_end_effectors.begin(),
                                          m_end_effectors.end(),
                                          [&](const auto& end_effector) {
                                            return end_effector->group()->getEndEffectorName() ==
                                                   name;
                                          });
        find_it != m_end_effectors.end())
    {
      return *(*find_it);
    }

    throw std::runtime_error{fmt::format("There is no end effector named '{}'", name)};
  }

  if (m_end_effectors.size() == 1)
  {
    const auto& default_ee = *m_end_effectors[0];
    RCLCPP_INFO(m_log,
                "Defaulting to end effector '%s' (group '%s')",
                default_ee.group()->getEndEffectorName().c_str(),
                default_ee.name().c_str());
    return default_ee;
  }
  else
  {
    throw std::runtime_error{
      "Cannot choose default end effector as there are multiple possible groups defined"};
  }
}

} // namespace manipulation_pipeline
