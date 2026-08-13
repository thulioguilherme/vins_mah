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

#ifndef VINS_MAH_FEATURE_TRACKER_H_
#define VINS_MAH_FEATURE_TRACKER_H_

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <unordered_set>

#include "vins_mah/camera_model/pinhole_camera.h"
#include "vins_mah/vio_options.h"

#include "vins_mah/utils/tic_toc.h"

#include <glog/logging.h>

using namespace std;
using namespace Eigen;

namespace vins_mah {

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
void reduceVector(std::vector<int> &v, std::vector<uchar> status);

// added by myself
void reduceVector(std::vector<float> &v, std::vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker(const vins_mah::VioOptions &vio_options);

    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 11, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    cv::Mat getMaskImage();

    void getCamerasIntrinsics(const vins_mah::VioOptions &options);

    void showUndistortion();
    void rejectWithF();
    std::vector<cv::Point2f> undistortedPts(std::vector<cv::Point2f> &pts, const std::shared_ptr<vins_mah::camera_model::PinholeCamera> &cam);
    std::vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, std::map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2,
        vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
        vector<int> &curLeftIds,
        vector<cv::Point2f> &curLeftPts, 
        vector<cv::Point2f> &curRightPts,
        map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int img_rows_; // height
    int img_cols_; // width
    double fx_;
    double fy_;
    int num_pts_;
    int min_pixel_dist_;
    double ransac_reproj_threshold_;
    bool use_stereo_;
    bool publish_track_images_;
    bool use_flow_back_;
    bool use_mask_;
    cv::Mat mask_img_;

    cv::Mat imTrack;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    std::vector<cv::Point2f> n_pts;
    std::vector<cv::Point2f> predict_pts;
    std::vector<cv::Point2f> predict_pts_debug;
    std::vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    std::vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    std::vector<cv::Point2f> pts_velocity, right_pts_velocity;
    std::vector<int> ids, ids_right;
    std::vector<int> track_cnt;
    std::map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    std::map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    std::map<int, cv::Point2f> prevLeftPtsMap;

    std::vector<std::shared_ptr<vins_mah::camera_model::PinholeCamera>> camera_models_;

    double cur_time;
    double prev_time;
    int n_id;
    bool hasPrediction;

    // added by myself
    void saveImage(const int frameCnt, const set<int> keepIndex, const set<int> removeIndex);
    Eigen::Matrix2d computeHessian(const cv::Mat& image, const cv::Point2f& center, int winSize = 21);
    /* void getCovariance(const cv::Mat& cur_img, const cv::Mat& prev_img, const cv::Point2f& center, const int id, const double err, const bool right); */
    void getCovariance(const cv::Mat& cur_img, const cv::Mat& prev_img, const cv::Point2f& center, const int id, const int feature_index, const double err, const bool right);
    void eraseMap(std::map<int,double>& flowVars, const std::vector<int>& ids);
    void getUndistortedCovs(const int& feature_id, const int feature_index, bool right);
    double computeVariance(const cv::Mat& cur_img, const cv::Mat& prev_img,
        const cv::Point2f& center, int winSize = 21);

    double inverse_fx_;
    double inverse_fy_;
    std::map<int, double> flowVars;
    std::map<int, std::array<Eigen::Matrix2d, 2>> posCovs;
    std::map<int, std::array<Eigen::Matrix2d, 2>> un_posCovs;
};

} // namespace vins_mah

#endif // VINS_MAH_FEATURE_TRACKER_H_
