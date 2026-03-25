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
#include "manipulation_pipeline/planning_steps/manipulation_planning_step.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation_pipeline {

std::vector<Eigen::Isometry3d> ManipulationPlanningStepBase::convertLinearMotion(
  const manipulation_pipeline_interfaces::msg::LinearMotion& msg,
  const Eigen::Isometry3d& offset) const
{
  std::vector<Eigen::Isometry3d> waypoints;

  switch (msg.motion_type)
  {
    case manipulation_pipeline_interfaces::msg::LinearMotion::MOTION_TYPE_LINEAR: {
      const auto dist = (msg.distance == 0.0) ? 0.1 : msg.distance;
      waypoints.push_back(offset * Eigen::Isometry3d{Eigen::Translation3d{0, 0, -dist}});
      break;
    }

    case manipulation_pipeline_interfaces::msg::LinearMotion::MOTION_TYPE_PATH: {
      Eigen::Isometry3d buf;
      waypoints.reserve(msg.waypoints.size());
      std::transform(msg.waypoints.begin(),
                     msg.waypoints.end(),
                     std::back_inserter(waypoints),
                     [&](const auto& msg) {
                       tf2::convert(msg, buf);
                       return offset * buf;
                     });
      break;
    }

    default:
      throw std::runtime_error{fmt::format("Unknon linear motion type {}", msg.motion_type)};
  }

  return waypoints;
}

} // namespace manipulation_pipeline
