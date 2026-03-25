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

/*!\file manipulation_pipeline/ik_sampler.h
 * \brief Sample IK configurations
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2026-03-25
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_IK_SAMPLER_H_INCLUDED
#define MANIPULATION_PIPELINE_IK_SAMPLER_H_INCLUDED

#include <Eigen/Geometry>
#include <boost/random/sobol.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/logger.hpp>

namespace manipulation_pipeline {

/*! \brief Sample a number of inverse kinematics solutions
 *
 * This is used for manipulation planning, as we require a set of possible ik solutions for the
 * target object pose.
 */
class IkSampler
{
public:
  /*! \brief Create new sampler
   *
   * \param reference_state Robot state that will be used to generate new samples
   * \param joint_group Group of joints to sample from
   * \param target_pose Pose to sample IK solutions for
   * \param max_samples Maximum number of samples to generate before terminating
   * \param log Logger to use for debug messages
   */
  IkSampler(moveit::core::RobotState reference_state,
            const moveit::core::JointModelGroup* joint_group,
            Eigen::Isometry3d target_pose,
            std::size_t max_samples,
            rclcpp::Logger log);

  /*! \brief Check if more samples should be taken based on constructor termination conditions
   *
   * \returns Whether sampling is done
   */
  [[nodiscard]] bool done() const;

  /*! \brief Sample a new configuration
   *
   * This only returns a sample if it is different from previous samples.
   */
  std::optional<moveit::core::RobotState> sample();

  /*! \brief Sample all required states
   *
   * This is just a shorthand for calling sample() until done().
   */
  std::vector<moveit::core::RobotState> sampleAll();

private:
  bool isNew(const moveit::core::RobotState& state, double eps) const;

  rclcpp::Logger m_log;

  boost::random::sobol_engine<std::uint32_t, 32> m_engine;
  boost::random::uniform_01<double> m_distribution;

  moveit::core::RobotState m_reference_state;
  moveit::core::RobotState m_sample_state;
  const moveit::core::JointModelGroup* m_joint_group;

  Eigen::Isometry3d m_target_pose;

  std::vector<moveit::core::RobotState> m_sampled_states;

  std::size_t m_max_samples;
  std::size_t m_sample_cnt = 0;

  std::size_t m_n_ik_error   = 0;
  std::size_t m_n_not_unique = 0;
};

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_IK_SAMPLER_H_INCLUDED
