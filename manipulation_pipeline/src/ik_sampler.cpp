// Copyright 2026 FZI Forschungszentrum Informatik
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
 * \date    2026-03-25
 *
 */
//----------------------------------------------------------------------
#include "manipulation_pipeline/ik_sampler.h"

#include <rclcpp/logging.hpp>

namespace manipulation_pipeline {

IkSampler::IkSampler(moveit::core::RobotState reference_state,
                     const moveit::core::JointModelGroup* joint_group,
                     Eigen::Isometry3d target_pose,
                     std::size_t max_samples,
                     rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_engine{joint_group->getActiveVariableCount()}
  , m_reference_state{std::move(reference_state)}
  , m_sample_state{m_reference_state}
  , m_joint_group{joint_group}
  , m_target_pose{std::move(target_pose)}
  , m_max_samples{max_samples}
{
}

bool IkSampler::done() const
{
  return m_sample_cnt >= m_max_samples;
}

std::optional<moveit::core::RobotState> IkSampler::sample()
{
  ++m_sample_cnt;

  const auto& joint_models = m_joint_group->getActiveJointModels();
  for (std::size_t i = 0; i < m_joint_group->getActiveVariableCount(); ++i)
  {
    const auto& bound = joint_models[i]->getVariableBounds()[0];
    const double s_i  = m_distribution(m_engine);
    const double p_i  = bound.min_position_ + s_i * (bound.max_position_ - bound.min_position_);
    m_sample_state.setJointPositions(joint_models[i], &p_i);
  }

  if (m_sample_state.setFromIK(m_joint_group, m_target_pose))
  {
    if (isNew(m_sample_state, 0.1))
    {
      m_sampled_states.push_back(m_sample_state);
      return m_sample_state;
    }
    else
    {
      ++m_n_not_unique;
    }
  }
  else
  {
    ++m_n_ik_error;
  }

  return std::nullopt;
}

std::vector<moveit::core::RobotState> IkSampler::sampleAll()
{
  std::vector<moveit::core::RobotState> samples;
  while (!done())
  {
    if (const auto new_sample = sample(); new_sample)
    {
      samples.push_back(std::move(*new_sample));
    }
  }

  RCLCPP_INFO(m_log,
              "Created %zu IK samples (%zu IK errors, %zu discarded)",
              samples.size(),
              m_n_ik_error,
              m_n_not_unique);

  return samples;
}

bool IkSampler::isNew(const moveit::core::RobotState& state, double eps) const
{
  const auto& joint_models = m_joint_group->getActiveJointModels();

  // The first sample is always new
  if (m_sampled_states.empty())
  {
    return true;
  }

  // Check against all prior sampled states
  return std::all_of(
    m_sampled_states.begin(), m_sampled_states.end(), [&](const moveit::core::RobotState& s) {
      // If there is a joint where the distance is larger then eps
      return std::any_of(
        joint_models.begin(), joint_models.end(), [&](const moveit::core::JointModel* joint) {
          const auto* j1 = state.getJointPositions(joint);
          const auto* j2 = s.getJointPositions(joint);

          for (std::size_t i = 0; i < joint->getVariableCount(); ++i)
          {
            if (std::abs(j1[i] - j2[i]) > eps)
            {
              return true;
            }
          }

          return false;
        });
    });
}

} // namespace manipulation_pipeline
