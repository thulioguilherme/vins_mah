/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "vins_mah/utils/rotations.h"

namespace vins_mah
{
  namespace utils
  {
    Eigen::Matrix3d Rotations::g2R(const Eigen::Vector3d &g)
    {
      Eigen::Matrix3d R0;
      Eigen::Vector3d ng1 = g.normalized();
      Eigen::Vector3d ng2{0, 0, 1.0};
      R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
      double yaw = Rotations::R2ypr(R0).x();
      R0 = Rotations::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
      // R0 = Rotations::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
      return R0;
    }

  } // namespace utils

} // namespace vins_mah
