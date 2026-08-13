/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef VINS_MAH_FACTOR_POSE_LOCAL_PARAMETERIZATION_H_
#define VINS_MAH_FACTOR_POSE_LOCAL_PARAMETERIZATION_H_

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include "vins_mah/utils/rotations.h"

namespace vins_mah
{
  namespace factor
  {

    class PoseLocalParameterization : public ceres::LocalParameterization
    {
      virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
      virtual bool ComputeJacobian(const double *x, double *jacobian) const;
      virtual int GlobalSize() const { return 7; };
      virtual int LocalSize() const { return 6; };
    };

  } // namespace factor

} // namespace vins_mah

#endif // VINS_MAH_FACTOR_POSE_LOCAL_PARAMETERIZATION_H_
