/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef VINS_MAH_FEATURE_MANAGER_H_
#define VINS_MAH_FEATURE_MANAGER_H_

#include <algorithm>
#include <array>
#include <list>
#include <vector>
#include <numeric>
using namespace std;

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
using namespace Eigen;

#include <glog/logging.h>

#include "vins_mah/vio_options.h"
#include "vins_mah/utils/tic_toc.h"

namespace vins_mah {

class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 11, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;

        // added by myself
        featureCovs[0] << _point(7), _point(8), _point(9), _point(10);
    }
    void rightObservation(const Eigen::Matrix<double, 11, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;

        // added by myself
        featureCovs[1] << _point(7), _point(8), _point(9), _point(10);
    }
    double cur_td;
    Vector3d point, pointRight;
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
    bool is_stereo;

    // added by myself
    std::array<Eigen::Matrix2d, 2> featureCovs;
};

class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    std::vector<FeaturePerFrame> feature_per_frame;
    int used_num;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class FeatureManager
{
  public:
    explicit FeatureManager(const vins_mah::VioOptions &options);

    void setRic(Matrix3d _ric[]);
    void clearState();
    int getFeatureCount();
    bool addFeatureCheckParallax(int frame_count, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 11, 1>>>> &image, double td);
    std::vector<std::pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth();
    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier(set<int> &outlierIndex);
    list<FeaturePerId> feature;
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;

    int window_size_;
    double min_parallax_;
    int num_cameras_;
    double focal_length_;
    bool use_stereo_;
    double init_depth_;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    /* const std::vector<Eigen::Matrix3d> &Rs_; */
    Eigen::Matrix3d ric[2];
};

} // namespace vins_mah

#endif // VINS_MAH_FEATURE_MANAGER_H_
