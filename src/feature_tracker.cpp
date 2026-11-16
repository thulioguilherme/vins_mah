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

#include "vins_mah/feature_tracker.h"

namespace vins_mah
{

  FeatureTracker::FeatureTracker(const vins_mah::VioOptions &vio_options) :
    img_rows_(vio_options.cameras[0].height),
    img_cols_(vio_options.cameras[0].width),
    fx_(vio_options.cameras[0].camera_model->K.at<double>(0, 0)),
    fy_(vio_options.cameras[0].camera_model->K.at<double>(1, 1)),
    num_pts_(vio_options.num_pts),
    min_pixel_dist_(vio_options.min_pixel_dist),
    ransac_reproj_threshold_(vio_options.ransac_reproj_threshold),
    use_stereo_(vio_options.use_stereo),
    publish_track_images_(vio_options.publish_track_images),
    use_flow_back_(vio_options.use_flow_back),
    use_mask_(vio_options.use_mask),
    mask_img_(vio_options.mask_img)
  {
    n_id = 0;
    hasPrediction = false;

    inverse_fx_ = 1 / fx_;
    inverse_fy_ = 1 / fy_;
  }

  void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++) {
      if (status[i]) {
        v[j++] = v[i];
      }
    }
    v.resize(j);
  }

  void reduceVector(vector<int> &v, vector<uchar> status)
  {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
      if (status[i])
        v[j++] = v[i];
    v.resize(j);
  }

  // added by myself
  void reduceVector(vector<float> &v, vector<uchar> status)
  {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
      if (status[i])
        v[j++] = v[i];
    v.resize(j);
  }

  bool FeatureTracker::inBorder(const cv::Point2f &pt) {
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < img_cols_ - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < img_rows_ - BORDER_SIZE;
  }


  cv::Mat FeatureTracker::getMaskImage() {
    cv::Mat mask = mask_img_.clone();

    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> tracked_features;

    for (size_t i = 0; i < cur_pts.size(); ++i) {
      tracked_features.emplace_back(track_cnt[i], std::make_pair(cur_pts[i], ids[i]));
    }

    std::sort(tracked_features.begin(), tracked_features.end(), [](const auto &a, const auto &b) {
      return a.first > b.first;
    });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (const auto &feature : tracked_features) {
      const cv::Point2f &point = feature.second.first;

      if (mask.at<uchar>(point) == 255) {
        cur_pts.push_back(point);
        ids.push_back(feature.second.second);
        track_cnt.push_back(feature.first);

        cv::circle(mask, point, min_pixel_dist_, 0, -1);
      }
    }

    return mask;
  }

  double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2) {
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
  }

  std::map<int, vector<pair<int, Eigen::Matrix<double, 11, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
  {
    vins_mah::utils::TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    cv::Mat rightImg = _img1;

    cur_pts.clear();

    // added by myself
    posCovs.clear();
    un_posCovs.clear();
    std::vector<float> left_err, right_err;

    if (prev_pts.size() > 0)
    {
      vins_mah::utils::TicToc t_o;
      std::vector<uchar> status;
      std::vector<float> err;
      if(hasPrediction)
      {
        cur_pts = predict_pts;
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, left_err, cv::Size(21, 21), 1,
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

        int succ_num = 0;
        for (size_t i = 0; i < status.size(); i++)
        {
          if (status[i])
            succ_num++;
        }
        if (succ_num < 10)
          cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, left_err, cv::Size(21, 21), 3);
      }
      else
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, left_err, cv::Size(21, 21), 3);
      // reverse check
      if (use_flow_back_) {
        vector<uchar> reverse_status;
        vector<cv::Point2f> reverse_pts = prev_pts;
        cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
        for(size_t i = 0; i < status.size(); i++)
        {
          if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
          {
            status[i] = 1;
          }
          else
            status[i] = 0;
        }
      }

      for (int i = 0; i < int(cur_pts.size()); i++)
        if (status[i] && !inBorder(cur_pts[i]))
          status[i] = 0;
      reduceVector(prev_pts, status);
      reduceVector(cur_pts, status);
      reduceVector(ids, status);
      reduceVector(track_cnt, status);
      /* VLOG(1) << "Temporal optical flow costs: " << t_o.toc() << "ms"; */
      //printf("track cnt %d\n", (int)ids.size());

      //added by myself
      reduceVector(left_err, status);
      eraseMap(flowVars, ids);
    }

    for (auto &n : track_cnt)
      n++;

    // added by myself
    for(size_t i = 0; i < cur_pts.size(); i++) {
      getCovariance(cur_img, prev_img, cur_pts[i], ids[i], i, false);
    }

    if (1)
    {
      //rejectWithF();
      /* VLOG(1) << "set mask begins"; */
      vins_mah::utils::TicToc t_m;
      cv::Mat mask_img = getMaskImage();
      /* VLOG(1) << "set mask costs " << t_m.toc() << "ms"; */

      /* VLOG(1) << "detect feature begins"; */
      vins_mah::utils::TicToc t_t;
      int n_max_cnt = num_pts_ - static_cast<int>(cur_pts.size());
      if (n_max_cnt > 0) {
        cv::goodFeaturesToTrack(cur_img, n_pts, num_pts_ - cur_pts.size(), 0.01, min_pixel_dist_, mask_img);
      } else {
        n_pts.clear();
      }
      /* VLOG(1) << "detect feature costs: " << t_t.toc() << " ms"; */

      for (auto &p : n_pts)
      {
        cur_pts.push_back(p);
        ids.push_back(n_id++);
        track_cnt.push_back(1);
      }
      //printf("feature cnt after add %d\n", (int)ids.size());
    }

    cur_un_pts = undistortedPts(cur_pts, camera_models_[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && use_stereo_)
    {
      ids_right.clear();
      cur_right_pts.clear();
      cur_un_right_pts.clear();
      right_pts_velocity.clear();
      cur_un_right_pts_map.clear();
      if(!cur_pts.empty())
      {
        //printf("stereo image; track feature on right image\n");
        vector<cv::Point2f> reverseLeftPts;
        vector<uchar> status, statusRightLeft;
        vector<float> err;
        // cur left ---- cur right
        cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, right_err, cv::Size(21, 21), 3);
        // reverse check cur right ---- cur left
        if (use_flow_back_)
        {
          cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
          for(size_t i = 0; i < status.size(); i++)
          {
            if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
              status[i] = 1;
            else
              status[i] = 0;
          }
        }

        ids_right = ids;
        reduceVector(cur_right_pts, status);
        reduceVector(ids_right, status);
        // only keep left-right point
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(cur_un_pts, status);
        reduceVector(pts_velocity, status);

        // added by myself
        reduceVector(right_err, status);

        cur_un_right_pts = undistortedPts(cur_right_pts, camera_models_[1]);
        right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
      }
      prev_un_right_pts_map = cur_un_right_pts_map;
    }

    if (publish_track_images_) {
      drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
    }

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for (size_t i = 0; i < cur_pts.size(); i++) {
      prevLeftPtsMap[ids[i]] = cur_pts[i];
    }

    // added by myself
    for (size_t i = 0; i < cur_right_pts.size(); i++) {
      getCovariance(rightImg, prev_img, cur_right_pts[i], ids_right[i], i, true);
    }

    for(size_t i = 0; i < ids.size(); i++) {
      getUndistortedCovs(ids[i], i, false);
      if (!_img1.empty() && use_stereo_)
        getUndistortedCovs(ids_right[i], i, true);
    }

    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 11, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++) {
      if(!std::isinf(cur_un_pts[i].x) && !std::isinf(cur_un_pts[i].y) &&
          !std::isnan(cur_un_pts[i].x) && !std::isnan(cur_un_pts[i].y))
      {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        // added by myself
        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        const auto cov = un_posCovs.find(feature_id);
        if (cov != un_posCovs.end())
          covariance = cov->second[0];
        // LOG(INFO) << "left id: %d; track count: %d,", feature_id, track_cnt[i];

        /* std::cout << "cur_un_pts: " */
        /*           << cur_un_pts[i].x << " " */
        /*           << cur_un_pts[i].y << std::endl; */

        /* std::cout << "cur_pts: " */
        /*           << cur_pts[i].x << " " */
        /*           << cur_pts[i].y << std::endl; */

        Eigen::Matrix<double, 11, 1> xyz_uv_velocity;
        // modified by myself
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y,
            covariance(0, 0), covariance(0, 1), covariance(1, 0), covariance(1, 1);
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
      }
    }

    if (!_img1.empty() && use_stereo_)
    {
      for (size_t i = 0; i < ids_right.size(); i++)
      {
        if(!std::isinf(cur_un_pts[i].x) && !std::isinf(cur_un_pts[i].y) &&
            !std::isnan(cur_un_right_pts[i].x) && !std::isnan(cur_un_right_pts[i].y))
        {
          int feature_id = ids_right[i];
          double x, y ,z;
          x = cur_un_right_pts[i].x;
          y = cur_un_right_pts[i].y;
          z = 1;
          double p_u, p_v;
          p_u = cur_right_pts[i].x;
          p_v = cur_right_pts[i].y;
          int camera_id = 1;
          double velocity_x, velocity_y;
          velocity_x = right_pts_velocity[i].x;
          velocity_y = right_pts_velocity[i].y;

          // added by myself
          Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
          const auto cov = un_posCovs.find(feature_id);
          if (cov != un_posCovs.end())
            covariance = cov->second[1];

          // LOG(INFO) << "right id: %d; track count: %d,", feature_id, track_cnt[i];

          Eigen::Matrix<double, 11, 1> xyz_uv_velocity;

          // modified by myself
          xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y,
              covariance(0, 0), covariance(0, 1), covariance(1, 0), covariance(1, 1);
          featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }   
      }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
  }

  void FeatureTracker::rejectWithF()
  {
    if (cur_pts.size() >= 8) {
      /* VLOG(1) << "FM ransac begins"; */
      vins_mah::utils::TicToc t_f;
      std::vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
      for (unsigned int i = 0; i < cur_pts.size(); i++) {
        Eigen::Vector3d tmp_p = camera_models_[0]->liftProjective(cur_pts[i]);

        tmp_p.x() = camera_models_[0]->K.at<double>(0, 0) * tmp_p.x() / tmp_p.z() + img_cols_ / 2.0;
        tmp_p.y() = camera_models_[0]->K.at<double>(1, 1) * tmp_p.y() / tmp_p.z() + img_rows_ / 2.0;
        un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

        tmp_p = camera_models_[0]->liftProjective(prev_pts[i]);
        tmp_p.x() = camera_models_[0]->K.at<double>(0, 0) * tmp_p.x() / tmp_p.z() + img_cols_ / 2.0;
        tmp_p.y() = camera_models_[0]->K.at<double>(1, 1) * tmp_p.y() / tmp_p.z() + img_rows_ / 2.0;
        un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
      }

      std::vector<uchar> status;
      cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, ransac_reproj_threshold_, 0.99, status);
      reduceVector(prev_pts, status);
      reduceVector(cur_pts, status);
      reduceVector(cur_un_pts, status);
      reduceVector(ids, status);
      reduceVector(track_cnt, status);
      /* VLOG(1) << "FM ransac: " << size_a << " - " << cur_pts.size() << ":" << 1.0 * cur_pts.size() / size_a; */
      /* VLOG(1) << "FM ransac costs: " << t_f.toc() << "ms"; */
    }
  }

  void FeatureTracker::showUndistortion() {
    cv::Mat undistortedImg(img_rows_ + 600, img_cols_ + 600, CV_8UC1, cv::Scalar(0));
    std::vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < img_cols_; i++)
      for (int j = 0; j < img_rows_; j++)
      {
        cv::Point2f a(i, j);
        Eigen::Vector3d b = camera_models_[0]->liftProjective(a);
        distortedp.emplace_back(a.x, a.y);
        undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
        //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
      }
    for (int i = 0; i < int(undistortedp.size()); i++) {
      cv::Mat pp(3, 1, CV_32FC1);
      pp.at<float>(0, 0) = undistortedp[i].x() * fx_ + img_cols_ / 2;
      pp.at<float>(1, 0) = undistortedp[i].y() * fy_ + img_rows_ / 2;
      pp.at<float>(2, 0) = 1.0;
      //cout << trackerData[0].K << endl;
      //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
      //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
      if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < img_rows_ + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < img_cols_ + 600)
      {
        undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
      }
      else
      {
      }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
  }

  std::vector<cv::Point2f> FeatureTracker::undistortedPts(std::vector<cv::Point2f> &pts, const std::shared_ptr<vins_mah::camera_model::PinholeCamera> &cam) {
    std::vector<cv::Point2f> undistorted_pts;
    if (pts.empty())
      return undistorted_pts;
    cv::undistortPoints(pts, undistorted_pts, cam->K, cam->D);

    return undistorted_pts;
  }

  vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
      map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
  {
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
      cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
      double dt = cur_time - prev_time;

      for (unsigned int i = 0; i < pts.size(); i++)
      {
        std::map<int, cv::Point2f>::iterator it;
        it = prev_id_pts.find(ids[i]);
        if (it != prev_id_pts.end())
        {
          double v_x = (pts[i].x - it->second.x) / dt;
          double v_y = (pts[i].y - it->second.y) / dt;
          pts_velocity.push_back(cv::Point2f(v_x, v_y));
        }
        else
          pts_velocity.push_back(cv::Point2f(0, 0));

      }
    }
    else
    {
      for (unsigned int i = 0; i < cur_pts.size(); i++)
      {
        pts_velocity.push_back(cv::Point2f(0, 0));
      }
    }
    return pts_velocity;
  }

  void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
      vector<int> &curLeftIds,
      vector<cv::Point2f> &curLeftPts, 
      vector<cv::Point2f> &curRightPts,
      map<int, cv::Point2f> &prevLeftPtsMap)
  {
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && use_stereo_)
      cv::hconcat(imLeft, imRight, imTrack);
    else
      imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
      double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
      cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && use_stereo_)
    {
      for (size_t i = 0; i < curRightPts.size(); i++)
      {
        cv::Point2f rightPt = curRightPts[i];
        rightPt.x += cols;
        cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
      }
    }

    std::map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
      int id = curLeftIds[i];
      mapIt = prevLeftPtsMap.find(id);
      if(mapIt != prevLeftPtsMap.end())
      {
        cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
      }
    }
  }


  void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts) {
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    std::map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++) {
      //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
      int id = ids[i];
      itPredict = predictPts.find(id);
      if (itPredict != predictPts.end()) {
        Eigen::Vector2d tmp_uv;
        tmp_uv = camera_models_[0]->spaceToPlane(itPredict->second);
        predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
      } else {
        predict_pts.push_back(prev_pts[i]);
      }
    }
  }


  void FeatureTracker::removeOutliers(set<int> &removePtsIds) {
    std::set<int>::iterator itSet;
    std::vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++) {
      itSet = removePtsIds.find(ids[i]);
      if (itSet != removePtsIds.end()) {
        status.push_back(0);
      } else {
        status.push_back(1);
      }
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
  }

  cv::Mat FeatureTracker::getTrackImage() {
    return imTrack;
  }

  // add by myself
  Eigen::Matrix2d FeatureTracker::computeHessian(const cv::Mat& image, const cv::Point2f& center, int winSize)  {
    // get the 21*21 patch centered at point.
    cv::Mat patch_f;
    cv::getRectSubPix(image, cv::Size(winSize, winSize), center, patch_f);

    if (patch_f.depth() != CV_64F) {
      patch_f.convertTo(patch_f, CV_64F);
    }

    // generate Gauss core
    double sigma = winSize * 0.5; 
    cv::Mat gk1 = cv::getGaussianKernel(winSize, sigma, CV_64F);
    cv::Mat gauss2d = gk1 * gk1.t();  // outer product

    cv::Mat Ix, Iy;
    cv::Sobel(patch_f, Ix, CV_64F, 1, 0, 3);
    cv::Sobel(patch_f, Iy, CV_64F, 0, 1, 3);

    cv::Mat Ix2 = Ix.mul(Ix).mul(gauss2d);
    cv::Mat Iy2 = Iy.mul(Iy).mul(gauss2d);
    cv::Mat Ixy = Ix.mul(Iy).mul(gauss2d);

    double Hxx = cv::sum(Ix2)[0];
    double Hyy = cv::sum(Iy2)[0];
    double Hxy = cv::sum(Ixy)[0];

    Eigen::Matrix2d H;
    H << Hxx, Hxy, Hxy, Hyy;

    return H;
  }

  void FeatureTracker::getCovariance(const cv::Mat& cur_img, const cv::Mat& prev_img, const cv::Point2f& center, const int id, const int feature_index, const bool right) {
    // if the feature only is tracked once and isn't in the right image, then stop.
    if (track_cnt[feature_index] < 2 && !right)
      return;

    // transform MAV into variance
    // constexpr double PI = std::acos(-1.0);
    // double var = (PI/2) * err * err;
    double var = computeVariance(cur_img, prev_img, center);

    // compute Hessian inverse
    Eigen::Matrix2d H = computeHessian(cur_img, center);
    Eigen::Matrix2d Hinv;
    double a = H(0,0), b = H(0,1), c = H(1,0), d = H(1,1);
    double det = a*d - b*c;
    if (std::abs(det) < 1e-12) {
      Hinv =  H.completeOrthogonalDecomposition().pseudoInverse();
    } else {
      Hinv <<  d/det, -b/det,
           -c/det,  a/det;
    }

    // var * H^-1 or (var + flowVar) * H^-1
    // get the flow variance of previous frame
    double flowVar = 0.0;
    auto it = flowVars.find(id);
    if (it != flowVars.end()) {
      flowVar = it->second;
    }
    // calculate the covariance
    // Eigen::Matrix2d Cov = (var + flowVar) * Hinv;
    Eigen::Matrix2d Cov = var * Hinv;
    // update the covariance
    // Each camera's covariance is computed once per frame; Eigen defaults are uninitialized.
    auto cov = posCovs.try_emplace(id,
        std::array<Eigen::Matrix2d, 2>{Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Zero()}).first;
    cov->second[right ? 1 : 0] = Cov;
    // update the flow variance
    if(!right)
      flowVars[id] = var + flowVar;
  }

  void FeatureTracker::eraseMap(std::map<int,double>& flowVars, const std::vector<int>& ids)
  {
    std::unordered_set<int> keep(ids.begin(), ids.end());

    for (auto it = flowVars.begin(); it != flowVars.end(); /* nothing */) {
      if (keep.find(it->first) == keep.end()) {
        it = flowVars.erase(it); 
      } else {
        ++it;
      }
    }
  }

  void FeatureTracker::getUndistortedCovs(const int& feature_id, const int feature_index, bool right) {
    Eigen::Matrix2d Kinv;
    Kinv << inverse_fx_, 0.0,
         0.0, inverse_fy_;

    if(track_cnt[feature_index] < 2 && !right)
        return;

    auto it = posCovs.find(feature_id);
    if (it != posCovs.end()) {
      auto cov = un_posCovs.try_emplace(feature_id,
          std::array<Eigen::Matrix2d, 2>{Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Zero()}).first;
      cov->second[right ? 1 : 0] = Kinv * it->second[right ? 1 : 0] * Kinv;
    }
  }

  void FeatureTracker::saveImage(const int frameCnt, const set<int> keepIndex, const set<int> removeIndex)
  {
    cv::Mat imSave = prev_img.clone();    
    cv::cvtColor(imSave, imSave, cv::COLOR_GRAY2RGB);

    for (const auto& f : prevLeftPtsMap) {
      if(removeIndex.find(f.first) != removeIndex.end()) {
        cv::circle(imSave, f.second, 2, cv::Scalar(0, 0, 255), 2);
        // cv::putText(imSave, cv::format("%d", f.first), cv::Point(f.second.x, f.second.y),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
      }
      else if(keepIndex.find(f.first) != keepIndex.end())   
        cv::circle(imSave, f.second, 2, cv::Scalar(0, 255, 0), 2);  
    }

    std::string savePath = "/mnt/d/temp_resources/imTracks/" + to_string(frameCnt) + ".png";
    cv::imwrite(savePath, imSave);
  }

  double FeatureTracker::computeVariance(const cv::Mat& cur_img, const cv::Mat& prev_img,
      const cv::Point2f& center, int winSize)
  {
    cv::Mat cur_f, prev_f;
    cur_img.convertTo(cur_f,  CV_32F);
    prev_img.convertTo(prev_f, CV_32F);

    cv::Mat patch_cur_f, patch_prev_f;
    cv::getRectSubPix(cur_f,  cv::Size(winSize, winSize), center, patch_cur_f);
    cv::getRectSubPix(prev_f, cv::Size(winSize, winSize), center, patch_prev_f);

    cv::Mat patch_cur_d, patch_prev_d;
    patch_cur_f.convertTo(patch_cur_d, CV_64F);
    patch_prev_f.convertTo(patch_prev_d, CV_64F);

    cv::Mat delta = patch_cur_d - patch_prev_d;  // CV_64F

    cv::Scalar s = cv::mean(delta);
    double meanDelta = s[0];

    cv::Mat tmp;
    cv::subtract(delta, meanDelta, tmp);  // tmp = delta - meanDelta
    cv::Mat sqr;
    cv::multiply(tmp, tmp, sqr);

    double sumsq = cv::sum(sqr)[0];
    int n = winSize * winSize;
    double varDelta = (n > 1) ? (sumsq / (n - 1)) : 0.0;

    return varDelta;
  }

  /* getCameraIntrinsics() //{ */

  void FeatureTracker::getCamerasIntrinsics(const vins_mah::VioOptions &options) {
    for (int i = 0; i < options.num_cameras; ++i) {
      auto camera_model = options.cameras[i].camera_model;

      camera_models_.push_back(camera_model);
    }
  }
  
  //}

} // namespace vins_mah
