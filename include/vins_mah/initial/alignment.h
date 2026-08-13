/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#ifndef VINS_MAH_INITIAL_ALIGNMENT_H_
#define VINS_MAH_INITIAL_ALIGNMENT_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

#include <glog/logging.h>

#include "vins_mah/vio_types.h"
#include "vins_mah/factor/integration_base.h"
#include "vins_mah/utils/rotations.h"

using namespace Eigen;
using namespace std;

namespace vins_mah
{
  namespace initial
  {

    class ImageFrame
    {
      public:
        ImageFrame(){};
        ImageFrame(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 11, 1>>>>& _points, double _t)
          :
            t{_t},
          is_key_frame{false}
        {
          points = _points;
        };

        std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 11, 1>> > > points;
        double t;
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        std::unique_ptr<vins_mah::factor::IntegrationBase> pre_integration;
        bool is_key_frame;
    };

    void solveGyroscopeBias(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, int window_size);

    bool visualImuAlignment(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &estimated_gravity, VectorXd &x, int window_size, Eigen::Vector3d gravity_reference, Eigen::Vector3d T_ItoC);

  } // namespace initial

} // namespace vins_mah

#endif // VINS_MAH_INITIAL_ALIGNMENT_H_
