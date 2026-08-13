/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef VINS_MAH_FACTOR_PROJECTION_H_
#define VINS_MAH_FACTOR_PROJECTION_H_

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "vins_mah/utils/rotations.h"
#include "vins_mah/utils/tic_toc.h"

namespace vins_mah
{
  namespace factor
  {

    class Projection : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
    {
      public:
        Projection(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
        void check(double **parameters);

        Eigen::Vector3d pts_i, pts_j;
        Eigen::Matrix<double, 2, 3> tangent_base;
        static Eigen::Matrix2d sqrt_info;
        static double sum_t;
    };

  } // namespace factor

} // namespace vins_mah

#endif // VINS_MAH_FACTOR_PROJECTION_H_
