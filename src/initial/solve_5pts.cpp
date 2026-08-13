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

#include "vins_mah/initial/solve_5pts.h"

using namespace cv;

namespace vins_mah
{
  namespace initial
  {

  bool MotionEstimator::solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rotation, Vector3d &Translation) {
    if (corres.size() >= 15) {
      vector<cv::Point2f> ll, rr;
      for (int i = 0; i < int(corres.size()); i++) {
        ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
        rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
      }

      cv::Mat mask;

      cv::Mat E = cv::findFundamentalMat(
          ll,
          rr,
          cv::RANSAC,
          0.3 / 460.0,
          0.99,
          mask);

      cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

      cv::Mat rot, trans;
      int inlier_cnt = cv::recoverPose(
          E,
          ll,
          rr,
          cameraMatrix,
          rot,
          trans,
          mask);

      Eigen::Matrix3d R;
      Eigen::Vector3d T;
      for (int i = 0; i < 3; i++) {
        T(i) = trans.at<double>(i, 0);
        for (int j = 0; j < 3; j++) {
          R(i, j) = rot.at<double>(i, j);
        }
      }

      Rotation = R.transpose();
      Translation = -R.transpose() * T;
      if (inlier_cnt > 12) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  }

} // namespace initial

} // namespace vins_mah
