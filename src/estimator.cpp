/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "vins_mah/estimator.h"

namespace vins_mah {

/* Estimator class constructor //{ */

Estimator::Estimator(const rclcpp::Node::SharedPtr &node, const std::string node_namespace, const vins_mah::VioOptions &vio_options)
  : node_(node), node_namespace_(node_namespace), vio_options_(vio_options), feature_manager_(vio_options)
{
  RCLCPP_INFO(node_->get_logger(), "Initializing Vins-mah estimator class!");
  estimated_gravity_ = vio_options.imu.gravity_reference;

  feature_tracker_ = std::make_shared<vins_mah::FeatureTracker>(vio_options_);

  initial_ex_rotation_ = std::make_shared<vins_mah::initial::ExRotation>(vio_options_.window_size);

  ps_.resize(vio_options_.window_size + 1);
  vs_.resize(vio_options_.window_size + 1);
  rs_.resize(vio_options_.window_size + 1);
  bas_.resize(vio_options_.window_size + 1);
  bgs_.resize(vio_options_.window_size + 1);

  headers_.resize(vio_options_.window_size + 1);

  pre_integrations_.resize(vio_options_.window_size + 1);

  dt_buf_.resize(vio_options_.window_size + 1);
  linear_acceleration_buf_.resize(vio_options_.window_size + 1);
  angular_velocity_buf_.resize(vio_options_.window_size + 1);

  para_pose_.resize(vio_options_.window_size + 1);
  para_speed_bias_.resize(vio_options_.window_size + 1);

  // Magic number from the original implementation
  // It is suppose to be bigger than NUM_PTS parameter
  para_feature_.resize(1000);

  last_path_ = Eigen::Vector3d(0, 0, 0);

  thread_initialized_ = false;

  // Create image transport
  image_transport::ImageTransport it(node_);
  it_publisher_track_ = it.advertise("~/trackhist", 1);

  // Setup publishers
  publisher_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("~/odom", 1);
  publisher_path_ = node_->create_publisher<nav_msgs::msg::Path>("~/path", 1);
  publisher_key_poses_ = node_->create_publisher<visualization_msgs::msg::Marker>("~/key_poses", 1);
  publisher_num_features_ = node_->create_publisher<std_msgs::msg::Float64>("~/num_points", 1);
  publisher_feature_landmarks_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/feature_landmarks", 1);
  publisher_marginalized_landmarks_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/marginalized_landmarks", 1);
  publisher_camera_visual_poses_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/camera_visual_poses", 1);

  camera_pose_visualizer_ = std::make_unique<vins_mah::utils::CameraPoseVisualization>(1.0f, 0.0f, 0.0f, 1.0f);
  camera_pose_visualizer_->setScale(0.1);
  camera_pose_visualizer_->setLineWidth(0.01);

  // Setup transform broadcaster
  transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  clearState();

  RCLCPP_INFO(node_->get_logger(), "Vins-mah estimator class initialized!");
}

//}

/* Estimator class desconstructor //{ */

Estimator::~Estimator() {
  if (vio_options_.use_multithreading) {
    process_thread_.join();
    RCLCPP_INFO(node_->get_logger(), "Joined processing thread!");
  }
}

//}

/* clearState() //{ */

void Estimator::clearState() {
  process_mutex_.lock();

  while(!accBuf.empty()) {
    accBuf.pop();
  }

  while(!gyrBuf.empty()) {
    gyrBuf.pop();
  }

  while(!featureBuf.empty()) {
    featureBuf.pop();
  }

  prevTime = -1;
  curTime = 0;
  openExEstimation = 0;
  initP = Eigen::Vector3d(0, 0, 0);
  initR = Eigen::Matrix3d::Identity();
  inputImageCnt = 0;
  initFirstPoseFlag = false;

  for (int i = 0; i < vio_options_.window_size + 1; i++) {
    rs_[i].setIdentity();
    ps_[i].setZero();
    vs_[i].setZero();
    bas_[i].setZero();
    bgs_[i].setZero();
    dt_buf_[i].clear();
    linear_acceleration_buf_[i].clear();
    angular_velocity_buf_[i].clear();

    pre_integrations_[i].reset();
  }

  for (int i = 0; i < vio_options_.num_cameras; i++) {
    tic[i] = Vector3d::Zero();
    ric[i] = Matrix3d::Identity();
  }

  first_imu = false,
  sum_of_back = 0;
  sum_of_front = 0;
  frame_count = 0;
  state_ = EstimatorState::INITIALIZATION;
  initial_timestamp = 0;
  all_image_frame.clear();

  tmp_pre_integration.reset();

  if (last_marginalization_info != nullptr) {
    delete last_marginalization_info;
    last_marginalization_info = nullptr;
  }

  tmp_pre_integration = nullptr;
  last_marginalization_parameter_blocks.clear();

  feature_manager_.clearState();

  failure_occur = 0;

  process_mutex_.unlock();
}

//}

/* setParameters() //{ */

void Estimator::setParameters() {
  process_mutex_.lock();
  for (int i = 0; i < vio_options_.num_cameras; ++i) {
    tic[i] = vio_options_.cameras[i].T_ItoC;
    ric[i] = vio_options_.cameras[i].R_ItoC;
    std::ostringstream ss;
    ss << "Extrinsic cam" << i << ":\n"
       << ric[i] << "\n"
       << tic[i].transpose();

    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
  }

  feature_manager_.setRic(ric);
  double focal_length = vio_options_.cameras[0].camera_model->K.at<double>(0, 0);
  vins_mah::factor::ProjectionTwoFrameOneCam::sqrt_info = focal_length / 1.5 * Matrix2d::Identity();
  vins_mah::factor::ProjectionTwoFrameTwoCam::sqrt_info = focal_length / 1.5 * Matrix2d::Identity();
  vins_mah::factor::ProjectionOneFrameTwoCam::sqrt_info = focal_length / 1.5 * Matrix2d::Identity();
  std::ostringstream ss;
  ss << "Initial gravity vector: " << estimated_gravity_;
  RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());

  feature_tracker_->getCamerasIntrinsics(vio_options_);

  RCLCPP_INFO(node_->get_logger(), "Using multithreading: %s", vio_options_.use_multithreading ? "True" : "False");

  if (vio_options_.use_multithreading && !thread_initialized_) {
    thread_initialized_ = true;
    process_thread_ = std::thread(&Estimator::processMeasurements, this);
  }

  process_mutex_.unlock();
}

//}

/* changeSensorType() //{ */

void Estimator::changeSensorType(bool use_imu, bool use_stereo) {
  bool restart = false;

  process_mutex_.lock();

  if (!vio_options_.use_imu && !vio_options_.use_stereo) { // ??
    RCLCPP_ERROR(node_->get_logger(), "Use at least one of the sensors (stereo or IMU)!");
  } else {
    if (vio_options_.use_imu != use_imu) {
      vio_options_.use_imu = use_imu;
      if (vio_options_.use_imu) {
        // reuse imu; restart system
        restart = true;
      } else {
        if (last_marginalization_info != nullptr) {
          delete last_marginalization_info;
        }

        tmp_pre_integration = nullptr;
        last_marginalization_info = nullptr;
        last_marginalization_parameter_blocks.clear();
      }
    }

    vio_options_.use_stereo = use_stereo;
  }

  RCLCPP_INFO(node_->get_logger(), "Using IMU: %s, using stereo: %s",
    vio_options_.use_imu ? "True" : "False",
    vio_options_.use_stereo ? "True" : "False");

  process_mutex_.unlock();

  if (restart) {
    clearState();
    setParameters();
  }
}

//}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1) {
    inputImageCnt++;
    std::map<int, std::vector<pair<int, Eigen::Matrix<double, 11, 1>>>> featureFrame;

    if (_img1.empty()) {
      featureFrame = feature_tracker_->trackImage(t, _img);
    } else {
      featureFrame = feature_tracker_->trackImage(t, _img, _img1);
    }

    if (vio_options_.publish_track_images) {
      cv::Mat track_img = feature_tracker_->getTrackImage();
      rclcpp::Time timestamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
      publishTrackImages(track_img, timestamp);
    }

    if (vio_options_.use_multithreading) {
      mBuf.lock();
      featureBuf.push(make_pair(t, featureFrame));
      mBuf.unlock();
    } else {
      mBuf.lock();
      featureBuf.push(make_pair(t, featureFrame));
      mBuf.unlock();
      vins_mah::utils::TicToc process_measurements_timer;
      processMeasurements();

      if (vio_options_.increase_verbosity) {
        RCLCPP_INFO(node_->get_logger(), "Process measurements time: %.3f ms", process_measurements_timer.toc());
      }
    }

}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity) {
  mBuf.lock();
  accBuf.push(make_pair(t, linearAcceleration));
  gyrBuf.push(make_pair(t, angularVelocity));
  //printf("input imu with time %f \n", t);
  mBuf.unlock();

  if (state_ == EstimatorState::OPTIMIZATION) {
    mPropagate.lock();
    fastPredictIMU(t, linearAcceleration, angularVelocity);
    /* pubLatestOdometry(latest_P, latest_Q, latest_V, t); */
    mPropagate.unlock();
  }
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 11, 1>>>> &featureFrame) {
  mBuf.lock();
  featureBuf.push(make_pair(t, featureFrame));
  mBuf.unlock();

  if (!vio_options_.use_multithreading) {
    processMeasurements();
  }
}

bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, vector<pair<double, Eigen::Vector3d>> &gyrVector) {
  if (accBuf.empty()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "Not receive IMU data");
    return false;
  }

  if (t1 <= accBuf.back().first) {
    while (accBuf.front().first <= t0) {
      accBuf.pop();
      gyrBuf.pop();
    }

    while (accBuf.front().first < t1) {
      accVector.push_back(accBuf.front());
      accBuf.pop();
      gyrVector.push_back(gyrBuf.front());
      gyrBuf.pop();
    }

    accVector.push_back(accBuf.front());
    gyrVector.push_back(gyrBuf.front());
  } else {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "Waiting for IMU data!");
    return false;
  }

  return true;
}

bool Estimator::IMUAvailable(double t) {
  if (!accBuf.empty() && t <= accBuf.back().first) {
    return true;
  } else {
    return false;
  }
}

void Estimator::processMeasurements() {
  while (1) {
    std::pair<double, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 11, 1> > > > > feature;
    std::vector<std::pair<double, Eigen::Vector3d>> accVector, gyrVector;
    if (!featureBuf.empty()) {
      feature = featureBuf.front();
      curTime = feature.first + vio_options_.cam_time_offset;
      while(1) {
        if ((!vio_options_.use_imu  || IMUAvailable(feature.first + vio_options_.cam_time_offset))) {
          break;
        } else {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "Waiting for imu data!");
          if (!vio_options_.use_multithreading) {
            return;
          }

        }
      }
      mBuf.lock();
      if (vio_options_.use_imu) {
        getIMUInterval(prevTime, curTime, accVector, gyrVector);
      }

      featureBuf.pop();
      mBuf.unlock();

      if (vio_options_.use_imu) {
        if (!initFirstPoseFlag) {
          initFirstIMUPose(accVector);
        }

        for(size_t i = 0; i < accVector.size(); i++) {
          double dt;
          if(i == 0)
            dt = accVector[i].first - prevTime;
          else if (i == accVector.size() - 1)
            dt = curTime - accVector[i - 1].first;
          else
            dt = accVector[i].first - accVector[i - 1].first;
          processIMU(dt, accVector[i].second, gyrVector[i].second);
        }
      }
      process_mutex_.lock();

      vins_mah::utils::TicToc timer;

      processImage(feature.second, feature.first);
      prevTime = curTime;

      double processing_time_ms = timer.toc();
      if (vio_options_.increase_verbosity) {
        printStatistics(processing_time_ms);
      }

      // Print camera extrinsics if we are calibrating it
      if (state_ == EstimatorState::OPTIMIZATION) {
        if (vio_options_.calib_cam_extrinsics_mode != ExtrinsicEstimationMode::FIXED) {
          for (int i = 0; i < vio_options_.num_cameras; ++i) {
            std::ostringstream ss;
            ss << "Online extrinsic calibration (Cam" << i << "): \n"
               << "  translation (tic): " << tic[i].transpose() << "\n"
               << "  rotation YPR (deg): " << vins_mah::utils::Rotations::R2ypr(ric[i]).transpose();

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "%s", ss.str().c_str());
          }
        }

        if (vio_options_.calib_cam_time_offset) {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
            "Estimated camera-IMU time offset (td): %.6f s", vio_options_.cam_time_offset);
        }
      }

      rclcpp::Time timestamp = rclcpp::Time(static_cast<int64_t>(feature.first * 1e9));
      publishOdometry(timestamp);
      publishTfs(timestamp);
      publishKeyPoses(timestamp);
      publishLandmarks(timestamp);
      publishCameraVisualPoses(timestamp);

      process_mutex_.unlock();
    }

    if (!vio_options_.use_multithreading) {
      break;
    }

  }
}

void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "Initializing IMU pose");
  initFirstPoseFlag = true;
  /* return; */
  Eigen::Vector3d averAcc(0, 0, 0);
  int n = (int)accVector.size();
  for (size_t i = 0; i < accVector.size(); i++) {
    averAcc = averAcc + accVector[i].second;
  }
  averAcc = averAcc / n;
  printf("Average acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
  Matrix3d R0 = vins_mah::utils::Rotations::g2R(averAcc);
  double yaw = vins_mah::utils::Rotations::R2ypr(R0).x();
  R0 = vins_mah::utils::Rotations::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  rs_[0] = R0;
  cout << "init R0 " << endl << rs_[0] << endl;
  //vs_[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r) {
  ps_[0] = p;
  rs_[0] = r;
  initP = p;
  initR = r;
}


void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity) {
  if (!first_imu) {
    first_imu = true;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
  }

  if (!pre_integrations_[frame_count]) {
    pre_integrations_[frame_count] = std::make_unique<vins_mah::factor::IntegrationBase>(acc_0, gyr_0, bas_[frame_count], bgs_[frame_count], vio_options_.imu);
  }

  if (frame_count != 0) {
    pre_integrations_[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    dt_buf_[frame_count].push_back(dt);
    linear_acceleration_buf_[frame_count].push_back(linear_acceleration);
    angular_velocity_buf_[frame_count].push_back(angular_velocity);

    int j = frame_count;
    Vector3d un_acc_0 = rs_[j] * (acc_0 - bas_[j]) - estimated_gravity_;
    Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - bgs_[j];
    rs_[j] *= vins_mah::utils::Rotations::deltaQ(un_gyr * dt).toRotationMatrix();
    Vector3d un_acc_1 = rs_[j] * (linear_acceleration - bas_[j]) - estimated_gravity_;
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    ps_[j] += dt * vs_[j] + 0.5 * dt * dt * un_acc;
    vs_[j] += dt * un_acc;
  }
  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 11, 1>>>> &image, const double header) {

  if (feature_manager_.addFeatureCheckParallax(frame_count, image, vio_options_.cam_time_offset)) {
    marginalization_strategy_ = MarginalizationStrategy::REMOVE_OLDEST;
  } else {
    marginalization_strategy_ = MarginalizationStrategy::REMOVE_SECOND_NEWEST;
  }

  std_msgs::msg::Float64 num_features;
  num_features.data = feature_manager_.getFeatureCount();
  publisher_num_features_->publish(num_features);

  headers_[frame_count] = header;

  vins_mah::initial::ImageFrame imageframe(image, header);
  imageframe.pre_integration = std::move(tmp_pre_integration);
  all_image_frame.insert(std::make_pair(header, std::move(imageframe)));
  tmp_pre_integration = std::make_unique<vins_mah::factor::IntegrationBase>(acc_0, gyr_0, bas_[frame_count], bgs_[frame_count], vio_options_.imu);

  if (vio_options_.calib_cam_extrinsics_mode != ExtrinsicEstimationMode::FIXED) {
    RCLCPP_INFO(node_->get_logger(), "Calibrating extrinsic parameters, rotation movement is needed!");
    if (frame_count != 0) {
      std::vector<std::pair<Vector3d, Vector3d>> corres = feature_manager_.getCorresponding(frame_count - 1, frame_count);
      Matrix3d calib_ric;
      if (initial_ex_rotation_->CalibrationExRotation(corres, pre_integrations_[frame_count]->delta_q, calib_ric)) {
        std::stringstream ss;
        RCLCPP_INFO(node_->get_logger(), "Initial extrinsic rotation: %s", ss.str().c_str());

        ric[0] = calib_ric;
        vio_options_.cameras[0].R_ItoC = calib_ric;
        vio_options_.calib_cam_extrinsics_mode = ExtrinsicEstimationMode::OPTIMIZE;
      }
    }
  }

  if (state_ == EstimatorState::INITIALIZATION) {
    // monocular + IMU initilization
    if (!vio_options_.use_stereo && vio_options_.use_imu) {
      if (frame_count == vio_options_.window_size) {
        bool result = false;
        if (vio_options_.calib_cam_extrinsics_mode != ExtrinsicEstimationMode::CALIBRATE && (header - initial_timestamp) > 0.1) {
          result = initialStructure();
          initial_timestamp = header;
        }

        if (result) {
          optimization();
          updateLatestStates();
          state_ = EstimatorState::OPTIMIZATION;
          slideWindow();
          RCLCPP_INFO(node_->get_logger(), "State estimator initialized");
        } else {
          slideWindow();
        }
      }
    }

    // stereo + IMU initilization
    if (vio_options_.use_stereo && vio_options_.use_imu) {
      feature_manager_.initFramePoseByPnP(frame_count, ps_.data(), rs_.data(), tic, ric);
      feature_manager_.triangulate(ps_.data(), rs_.data(), tic, ric);
      if (frame_count == vio_options_.window_size) {
        std::map<double, vins_mah::initial::ImageFrame>::iterator frame_it;
        int i = 0;
        for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++) {
          frame_it->second.R = rs_[i];
          frame_it->second.T = ps_[i];
          i++;
        }
        vins_mah::initial::solveGyroscopeBias(all_image_frame, bgs_.data(), vio_options_.window_size);
        for (int i = 0; i <= vio_options_.window_size; i++) {
          pre_integrations_[i]->repropagate(Vector3d::Zero(), bgs_[i]);
        }
        optimization();
        updateLatestStates();
        state_ = EstimatorState::OPTIMIZATION;
        slideWindow();
        RCLCPP_INFO(node_->get_logger(), "State estimator initialized");
      }
    }

    // stereo only initilization
    if (vio_options_.use_stereo && !vio_options_.use_imu) {
      feature_manager_.initFramePoseByPnP(frame_count, ps_.data(), rs_.data(), tic, ric);
      feature_manager_.triangulate(ps_.data(), rs_.data(), tic, ric);
      optimization();

      if (frame_count == vio_options_.window_size) {
        optimization();
        updateLatestStates();
        state_ = EstimatorState::OPTIMIZATION;
        slideWindow();
        RCLCPP_INFO(node_->get_logger(), "State estimator initialized");
      }
    }

    if (frame_count < vio_options_.window_size) {
      frame_count++;
      int prev_frame = frame_count - 1;
      ps_[frame_count] = ps_[prev_frame];
      vs_[frame_count] = vs_[prev_frame];
      rs_[frame_count] = rs_[prev_frame];
      bas_[frame_count] = bas_[prev_frame];
      bgs_[frame_count] = bgs_[prev_frame];
    }

  } else {
    vins_mah::utils::TicToc t_solve;
    if (!vio_options_.use_imu) {
      feature_manager_.initFramePoseByPnP(frame_count, ps_.data(), rs_.data(), tic, ric);
    }

    feature_manager_.triangulate(ps_.data(), rs_.data(), tic, ric);
    std::set<int> keepIndex, removeIndex;
    //outliersRejection(keepIndex, removeIndex);
    outliersRejection(removeIndex);
    feature_manager_.removeOutlier(removeIndex);
    // feature_tracker_->saveImage(inputImageCnt, keepIndex, removeIndex);
    optimization();
    // set<int> removeIndex;
    // outliersRejection(removeIndex);
    // feature_manager_.removeOutlier(removeIndex);
    // if (! MULTIPLE_THREAD)
    // {
    //     feature_tracker_->removeOutliers(removeIndex);
    //     predictPtsInNextFrame();
    // }

    /* RCLCPP_INFO(node_->get_logger(), "Solver costs: %fms", t_solve.toc()); */

    if (failureDetection()) {
      RCLCPP_WARN(node_->get_logger(), "Failure detection!");
      failure_occur = 1;
      clearState();
      setParameters();
      RCLCPP_WARN(node_->get_logger(), "System reboot!");
      return;
    }

    slideWindow();
    feature_manager_.removeFailures();
    // prepare output of VINS
    key_poses_.clear();
    for (int i = 0; i <= vio_options_.window_size; i++) {
      key_poses_.push_back(ps_[i]);
    }

    last_R = rs_[vio_options_.window_size];
    last_P = ps_[vio_options_.window_size];
    last_R0 = rs_[0];
    last_P0 = ps_[0];
    updateLatestStates();
  }
}

bool Estimator::initialStructure() {
  vins_mah::utils::TicToc t_sfm;

  //check imu observibility
  {
    std::map<double, vins_mah::initial::ImageFrame>::iterator frame_it;
    Vector3d sum_g;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
      double dt = frame_it->second.pre_integration->sum_dt;
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
      sum_g += tmp_g;
    }
    Eigen::Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
    double var = 0;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
      double dt = frame_it->second.pre_integration->sum_dt;
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
    }

    var = sqrt(var / ((int)all_image_frame.size() - 1));
    if (var < 0.25) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "Not enough IMU excitation (imu variation: %lf)", var);
      // return false;
    }
  }

  // global sfm
  std::vector<Eigen::Quaterniond> Q(frame_count + 1);
  std::vector<Eigen::Vector3d> T(frame_count + 1);
  std::map<int, Eigen::Vector3d> sfm_tracked_points;
  std::vector<vins_mah::initial::SFMFeature> sfm_f;
  for (auto &it_per_id : feature_manager_.feature) {
    int imu_j = it_per_id.start_frame - 1;
    vins_mah::initial::SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = it_per_id.feature_id;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      Vector3d pts_j = it_per_frame.point;
      tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
    }
    sfm_f.push_back(tmp_feature);
  }

  Eigen::Matrix3d relative_R;
  Eigen::Vector3d relative_T;
  int l;
  if (!relativePose(relative_R, relative_T, l)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "Not enough features or parallax. Move device around!");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Trying to start the global SFM");
  vins_mah::initial::GlobalSFM sfm;
  if (!sfm.construct(frame_count + 1, Q.data(), T.data(), l, relative_R, relative_T, sfm_f, sfm_tracked_points)) {
    RCLCPP_WARN(node_->get_logger(), "global SFM failed!");
    marginalization_strategy_ = MarginalizationStrategy::REMOVE_OLDEST;
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Global SFM initialized");

  //solve pnp for all frame
  std::map<double, vins_mah::initial::ImageFrame>::iterator frame_it;
  std::map<int, Vector3d>::iterator it;
  frame_it = all_image_frame.begin( );
  for (int i = 0; frame_it != all_image_frame.end( ); frame_it++) {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;
    if ((frame_it->first) == headers_[i]) {
      frame_it->second.is_key_frame = true;
      frame_it->second.R = Q[i].toRotationMatrix() * vio_options_.cameras[0].R_ItoC.transpose();
      frame_it->second.T = T[i];
      i++;
      continue;
    }
    if ((frame_it->first) > headers_[i]) {
      i++;
    }
    Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Eigen::Vector3d P_inital = - R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    std::vector<cv::Point3f> pts_3_vector;
    std::vector<cv::Point2f> pts_2_vector;
    for (auto &id_pts : frame_it->second.points) {
      int feature_id = id_pts.first;
      for (auto &i_p : id_pts.second) {
        it = sfm_tracked_points.find(feature_id);
        if (it != sfm_tracked_points.end()) {
          Vector3d world_pts = it->second;
          cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
          pts_3_vector.push_back(pts_3);
          Vector2d img_pts = i_p.second.head<2>();
          cv::Point2f pts_2(img_pts(0), img_pts(1));
          pts_2_vector.push_back(pts_2);
        }
      }
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (pts_3_vector.size() < 6) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Not enough points for solve PnP!");
      return false;
    }

    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Solve PnP failed!");
      return false;
    }

    cv::Rodrigues(rvec, r);
    Eigen::MatrixXd R_pnp,tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.R = R_pnp * vio_options_.cameras[0].R_ItoC.transpose();
    frame_it->second.T = T_pnp;
  }

  if (!visualInitialAlign()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,  "Misalign visual structure with IMU");
    return false;
  }

  return true;
}

bool Estimator::visualInitialAlign() {
  vins_mah::utils::TicToc t_g;
  VectorXd x;
  //solve scale
  bool result = vins_mah::initial::visualImuAlignment(all_image_frame, bgs_.data(), estimated_gravity_, x, vio_options_.window_size, vio_options_.imu.gravity_reference, vio_options_.cameras[0].T_ItoC);
  if (!result) {
    RCLCPP_WARN(node_->get_logger(), "Solve visual IMU alignemnt failed!");
    return false;
  }

  // change state
  for (int i = 0; i <= frame_count; i++) {
    Matrix3d Ri = all_image_frame[headers_[i]].R;
    Vector3d Pi = all_image_frame[headers_[i]].T;
    ps_[i] = Pi;
    rs_[i] = Ri;
    all_image_frame[headers_[i]].is_key_frame = true;
  }

  double s = (x.tail<1>())(0);
  for (int i = 0; i <= vio_options_.window_size; i++) {
    pre_integrations_[i]->repropagate(Vector3d::Zero(), bgs_[i]);
  }
  for (int i = frame_count; i >= 0; i--) {
    ps_[i] = s * ps_[i] - rs_[i] * vio_options_.cameras[0].T_ItoC - (s * ps_[0] - rs_[0] * vio_options_.cameras[0].T_ItoC);
  }
  int kv = -1;
  std::map<double, vins_mah::initial::ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
  {
    if(frame_i->second.is_key_frame)
    {
      kv++;
      vs_[kv] = frame_i->second.R * x.segment<3>(kv * 3);
    }
  }

  Eigen::Matrix3d R0 = vins_mah::utils::Rotations::g2R(estimated_gravity_);
  double yaw = vins_mah::utils::Rotations::R2ypr(R0 * rs_[0]).x();
  R0 = vins_mah::utils::Rotations::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  estimated_gravity_ = R0 * estimated_gravity_;
  //Matrix3d rot_diff = R0 * rs_[0].transpose();
  Matrix3d rot_diff = R0;
  for (int i = 0; i <= frame_count; i++) {
    ps_[i] = rot_diff * ps_[i];
    rs_[i] = rot_diff * rs_[i];
    vs_[i] = rot_diff * vs_[i];
  }
  std::stringstream ss;
  ss << "g0     " << estimated_gravity_.transpose();
  RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());

  ss.str("");
  ss.clear();

  ss << "my R0  " << vins_mah::utils::Rotations::R2ypr(rs_[0]).transpose();
  RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());

  feature_manager_.clearDepth();
  feature_manager_.triangulate(ps_.data(), rs_.data(), tic, ric);

  return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l) {
  // find previous frame which contians enough correspondance and parallex with newest frame
  for (int i = 0; i < vio_options_.window_size; i++) {
    std::vector<pair<Vector3d, Vector3d>> corres;
    corres = feature_manager_.getCorresponding(i, vio_options_.window_size);
    /* RCLCPP_INFO(node_->get_logger(), "Correspondences: %zu", corres.size()); */
    if (corres.size() > 20) {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++) {
        Vector2d pts_0(corres[j].first(0), corres[j].first(1));
        Vector2d pts_1(corres[j].second(0), corres[j].second(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;
      }
      average_parallax = 1.0 * sum_parallax / int(corres.size());
      bool ok = m_estimator.solveRelativeRT(corres, relative_R, relative_T);
      /* RCLCPP_INFO(node_->get_logger(), "recoverPose=%d", ok); */
      if (average_parallax * 460 > 30 && ok) {
        l = i;
        RCLCPP_INFO(node_->get_logger(), "average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
        return true;
      }
    }
  }
  return false;
}

void Estimator::vector2double() {
  for (int i = 0; i <= vio_options_.window_size; i++) {
    para_pose_[i][0] = ps_[i].x();
    para_pose_[i][1] = ps_[i].y();
    para_pose_[i][2] = ps_[i].z();
    Quaterniond q{rs_[i]};
    para_pose_[i][3] = q.x();
    para_pose_[i][4] = q.y();
    para_pose_[i][5] = q.z();
    para_pose_[i][6] = q.w();

    if (vio_options_.use_imu) {
      para_speed_bias_[i][0] = vs_[i].x();
      para_speed_bias_[i][1] = vs_[i].y();
      para_speed_bias_[i][2] = vs_[i].z();

      para_speed_bias_[i][3] = bas_[i].x();
      para_speed_bias_[i][4] = bas_[i].y();
      para_speed_bias_[i][5] = bas_[i].z();

      para_speed_bias_[i][6] = bgs_[i].x();
      para_speed_bias_[i][7] = bgs_[i].y();
      para_speed_bias_[i][8] = bgs_[i].z();
    }
  }

  for (int i = 0; i < vio_options_.num_cameras; i++) {
    para_Ex_Pose[i][0] = tic[i].x();
    para_Ex_Pose[i][1] = tic[i].y();
    para_Ex_Pose[i][2] = tic[i].z();
    Quaterniond q{ric[i]};
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
  }

  int feature_count = feature_manager_.getFeatureCount();

  VectorXd dep = feature_manager_.getDepthVector();

  para_feature_.resize(feature_count);

  for (int i = 0; i < feature_count; i++)
      para_feature_[i][0] = dep(i);

  para_Td[0][0] = vio_options_.cam_time_offset;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = vins_mah::utils::Rotations::R2ypr(rs_[0]);
    Vector3d origin_P0 = ps_[0];

    if (failure_occur)
    {
        origin_R0 = vins_mah::utils::Rotations::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if(vio_options_.use_imu)
    {
        Vector3d origin_R00 = vins_mah::utils::Rotations::R2ypr(Quaterniond(para_pose_[0][6],
                                                          para_pose_[0][3],
                                                          para_pose_[0][4],
                                                          para_pose_[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = vins_mah::utils::Rotations::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
          if (vio_options_.increase_verbosity) {
            RCLCPP_WARN(node_->get_logger(),"Euler singular point!");
          }
          rot_diff = rs_[0] * Quaterniond(para_pose_[0][6],
              para_pose_[0][3],
              para_pose_[0][4],
              para_pose_[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= vio_options_.window_size; i++)
        {

            rs_[i] = rot_diff * Quaterniond(para_pose_[i][6], para_pose_[i][3], para_pose_[i][4], para_pose_[i][5]).normalized().toRotationMatrix();

            ps_[i] = rot_diff * Vector3d(para_pose_[i][0] - para_pose_[0][0],
                                    para_pose_[i][1] - para_pose_[0][1],
                                    para_pose_[i][2] - para_pose_[0][2]) + origin_P0;


                vs_[i] = rot_diff * Vector3d(para_speed_bias_[i][0],
                                            para_speed_bias_[i][1],
                                            para_speed_bias_[i][2]);

                bas_[i] = Vector3d(para_speed_bias_[i][3],
                                  para_speed_bias_[i][4],
                                  para_speed_bias_[i][5]);

                bgs_[i] = Vector3d(para_speed_bias_[i][6],
                                  para_speed_bias_[i][7],
                                  para_speed_bias_[i][8]);

        }
    }
    else
    {
        for (int i = 0; i <= vio_options_.window_size; i++)
        {
            rs_[i] = Quaterniond(para_pose_[i][6], para_pose_[i][3], para_pose_[i][4], para_pose_[i][5]).normalized().toRotationMatrix();

            ps_[i] = Vector3d(para_pose_[i][0], para_pose_[i][1], para_pose_[i][2]);
        }
    }

    if(vio_options_.use_imu)
    {
        for (int i = 0; i < vio_options_.num_cameras; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    VectorXd dep = feature_manager_.getDepthVector();
    for (int i = 0; i < feature_manager_.getFeatureCount(); i++)
        dep(i) = para_feature_[i][0];
    feature_manager_.setDepth(dep);

    if(vio_options_.use_imu)
        vio_options_.cam_time_offset = para_Td[0][0];

}

bool Estimator::failureDetection() {
  return false; // ??
  if (feature_manager_.last_track_num < 2) {
    RCLCPP_INFO(node_->get_logger(), " little feature %d", feature_manager_.last_track_num);
    //return true;
  }
  if (bas_[vio_options_.window_size].norm() > 2.5) {
    RCLCPP_INFO(node_->get_logger(), " big IMU acc bias estimation %f", bas_[vio_options_.window_size].norm());
    return true;
  }
  if (bgs_[vio_options_.window_size].norm() > 1.0) {
    RCLCPP_INFO(node_->get_logger(), " big IMU gyr bias estimation %f", bgs_[vio_options_.window_size].norm());
    return true;
  }
  /*
     if (tic(0) > 1)
     {
     RCLCPP_INFO(node_->get_logger(), " big extri param estimation %d", tic(0) > 1);
     return true;
     }
     */
  Vector3d tmp_P = ps_[vio_options_.window_size];
  if ((tmp_P - last_P).norm() > 5)
  {
    //RCLCPP_INFO(node_->get_logger(), " big translation");
    //return true;
  }
  if (abs(tmp_P.z() - last_P.z()) > 1)
  {
    //RCLCPP_INFO(node_->get_logger(), " big z translation");
    //return true;
  }
  Matrix3d tmp_R = rs_[vio_options_.window_size];
  Matrix3d delta_R = tmp_R.transpose() * last_R;
  Quaterniond delta_Q(delta_R);
  double delta_angle;
  delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
  if (delta_angle > 50)
  {
    RCLCPP_INFO(node_->get_logger(), " big delta_angle ");
    //return true;
  }
  return false;
}

void Estimator::optimization() {
  vins_mah::utils::TicToc optimization_timer, ceres_preparation_timer;
  vector2double();

  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  loss_function = new ceres::HuberLoss(1.0);

  for (int i = 0; i < frame_count + 1; i++) {
    ceres::LocalParameterization *local_parameterization = new vins_mah::factor::PoseLocalParameterization();
    problem.AddParameterBlock(para_pose_[i].data(), StateSize::POSE, local_parameterization);
    if (vio_options_.use_imu) {
      problem.AddParameterBlock(para_speed_bias_[i].data(), StateSize::SPEED_BIAS);
    }
  }

  if (!vio_options_.use_imu) {
    problem.SetParameterBlockConstant(para_pose_[0].data());
  }

  for (int i = 0; i < vio_options_.num_cameras; i++) {
    ceres::LocalParameterization *local_parameterization = new vins_mah::factor::PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose[i], StateSize::POSE, local_parameterization);

    bool calib_cam_extrinsics = (vio_options_.calib_cam_extrinsics_mode == ExtrinsicEstimationMode::OPTIMIZE || vio_options_.calib_cam_extrinsics_mode == ExtrinsicEstimationMode::CALIBRATE);

    if ((calib_cam_extrinsics && frame_count == vio_options_.window_size && vs_[0].norm() > 0.2) || openExEstimation) {
      //RCLCPP_INFO(node_->get_logger(), "estimate extinsic param");
      openExEstimation = 1;
    } else {
      //RCLCPP_INFO(node_->get_logger(), "fix extinsic param");
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
  }
  problem.AddParameterBlock(para_Td[0], 1);

  bool calib_cam_extrinsics = (vio_options_.calib_cam_extrinsics_mode == ExtrinsicEstimationMode::OPTIMIZE || vio_options_.calib_cam_extrinsics_mode == ExtrinsicEstimationMode::CALIBRATE);
  if (!calib_cam_extrinsics || vs_[0].norm() < 0.2) {
    problem.SetParameterBlockConstant(para_Td[0]);
  }

  if (last_marginalization_info && last_marginalization_info->valid) {
    // construct new marginlization_factor
    vins_mah::factor::Marginalization *marginalization_factor = new vins_mah::factor::Marginalization(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks);
  }

  if (vio_options_.use_imu) {
    for (int i = 0; i < frame_count; i++) {
      int j = i + 1;
      if (pre_integrations_[j]->sum_dt > 10.0) {
        continue;
      }
      vins_mah::factor::Imu* imu_factor = new vins_mah::factor::Imu(pre_integrations_[j].get(), vio_options_.imu.gravity_reference);
      problem.AddResidualBlock(imu_factor, NULL, para_pose_[i].data(), para_speed_bias_[i].data(), para_pose_[j].data(), para_speed_bias_[j].data());
    }
  }

  int f_m_cnt = 0;
  int feature_index = -1;
  for (auto &it_per_id : feature_manager_.feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4) {
      continue;
    }

    ++feature_index;

    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = it_per_frame.point;
        vins_mah::factor::ProjectionTwoFrameOneCam *f_td = new vins_mah::factor::ProjectionTwoFrameOneCam(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
        problem.AddResidualBlock(f_td, loss_function, para_pose_[imu_i].data(), para_pose_[imu_j].data(), para_Ex_Pose[0], para_feature_[feature_index].data(), para_Td[0]);
      }

      if(vio_options_.use_stereo && it_per_frame.is_stereo) {
        Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          vins_mah::factor::ProjectionTwoFrameTwoCam *f = new vins_mah::factor::ProjectionTwoFrameTwoCam(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          problem.AddResidualBlock(f, loss_function, para_pose_[imu_i].data(), para_pose_[imu_j].data(), para_Ex_Pose[0], para_Ex_Pose[1], para_feature_[feature_index].data(), para_Td[0]);
        } else {
          vins_mah::factor::ProjectionOneFrameTwoCam *f = new vins_mah::factor::ProjectionOneFrameTwoCam(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_feature_[feature_index].data(), para_Td[0]);
        }

      }
      f_m_cnt++;
    }
  }

  if (vio_options_.increase_verbosity) {
    RCLCPP_INFO(node_->get_logger(), "Visual measurement counter: %d", f_m_cnt);
    RCLCPP_INFO(node_->get_logger(), "Ceres preparation timer: %.3f ms", ceres_preparation_timer.toc());
  }

  ceres::Solver::Options options;

  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = vio_options_.solver_max_num_iterations;

  if (marginalization_strategy_ == MarginalizationStrategy::REMOVE_OLDEST) {
    options.max_solver_time_in_seconds = vio_options_.solver_max_time * 4.0 / 5.0;
  } else {
    options.max_solver_time_in_seconds = vio_options_.solver_max_time;
  }

  vins_mah::utils::TicToc ceres_solver_timer;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (vio_options_.increase_verbosity) {
    RCLCPP_INFO(node_->get_logger(), "Ceres solver num. of iterations: %d", static_cast<int>(summary.iterations.size()));
    RCLCPP_INFO(node_->get_logger(), "Ceres solver timer: %.3f ms", ceres_solver_timer.toc());
  }

  // added by myself
  std::vector<int> frame_list;
  if (frame_count == vio_options_.window_size) {
    if (marginalization_strategy_ == MarginalizationStrategy::REMOVE_OLDEST) {
      frame_list = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    }
    else {
      frame_list = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    }
  } else {
    for (int i = 0; i < frame_count; i++) {
      frame_list.push_back(i);
    }
  }

  if (frame_list.size() > 0) {
    ceres::Problem::EvaluateOptions eval_opts;
    eval_opts.apply_loss_function = true;
    std::vector<double*> param_blocks;
    problem.GetParameterBlocks(&param_blocks);
    eval_opts.parameter_blocks = param_blocks;
    ceres::CRSMatrix J_crs;
    problem.Evaluate(eval_opts, nullptr, nullptr, nullptr, &J_crs);

    // Convert CRS to Eigen
    Eigen::SparseMatrix<double> J(J_crs.num_rows, J_crs.num_cols);
    for (int r = 0; r < J_crs.num_rows; ++r) {
      for (int k = J_crs.rows[r]; k < J_crs.rows[r + 1]; ++k) {
        int c = J_crs.cols[k];
        J.insert(r, c) = J_crs.values[k];
      }
    }
    J.makeCompressed();  // Optional: for performance

    // obtain Hessian Matrix
    Eigen::MatrixXd H_total = J.transpose() * J;

    // store the covariances of UAV's pose
    /* for (size_t i = 0; i < frame_list.size(); i++) { */
    /*   int block_start = frame_list[i] * 6; */
    /*   pose_covs_[i] = Hinv_pp.block(block_start, block_start, 6, 6); */
    /* } */
  }

  double2vector();

  if (frame_count < vio_options_.window_size) {
    return;
  }

  vins_mah::utils::TicToc marginalization_timer;
  if (marginalization_strategy_ == MarginalizationStrategy::REMOVE_OLDEST) {
    vins_mah::factor::MarginalizationInfo *marginalization_info = new vins_mah::factor::MarginalizationInfo();
    vector2double();

    if (last_marginalization_info && last_marginalization_info->valid) {
      std::vector<int> drop_set;
      for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
        if (last_marginalization_parameter_blocks[i] == para_pose_[0].data() ||
            last_marginalization_parameter_blocks[i] == para_speed_bias_[0].data()) {
          drop_set.push_back(i);
        }
      }
      // construct new marginlization_factor
      vins_mah::factor::Marginalization *marginalization_factor = new vins_mah::factor::Marginalization(last_marginalization_info);
      vins_mah::factor::ResidualBlockInfo *residual_block_info = new vins_mah::factor::ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);
      marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    if (vio_options_.use_imu) {
      if (pre_integrations_[1]->sum_dt < 10.0) {
        vins_mah::factor::Imu* imu_factor = new vins_mah::factor::Imu(pre_integrations_[1].get(), vio_options_.imu.gravity_reference);
        vins_mah::factor::ResidualBlockInfo *residual_block_info = new vins_mah::factor::ResidualBlockInfo(imu_factor, NULL, std::vector<double *>{para_pose_[0].data(), para_speed_bias_[0].data(), para_pose_[1].data(), para_speed_bias_[1].data()}, std::vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    {
      int feature_index = -1;
      for (auto &it_per_id : feature_manager_.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4) {
          continue;
        }

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        if (imu_i != 0) {
          continue;
        }

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
          imu_j++;
          if(imu_i != imu_j) {
            Vector3d pts_j = it_per_frame.point;
            vins_mah::factor::ProjectionTwoFrameOneCam *f_td = new vins_mah::factor::ProjectionTwoFrameOneCam(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
            vins_mah::factor::ResidualBlockInfo *residual_block_info = new vins_mah::factor::ResidualBlockInfo(f_td, loss_function, std::vector<double *>{para_pose_[imu_i].data(), para_pose_[imu_j].data(), para_Ex_Pose[0], para_feature_[feature_index].data(), para_Td[0]}, std::vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }
          if (vio_options_.use_stereo && it_per_frame.is_stereo) {
            Vector3d pts_j_right = it_per_frame.pointRight;
            if (imu_i != imu_j) {
              vins_mah::factor::ProjectionTwoFrameTwoCam *f = new vins_mah::factor::ProjectionTwoFrameTwoCam(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
              vins_mah::factor::ResidualBlockInfo *residual_block_info = new vins_mah::factor::ResidualBlockInfo(f, loss_function,
                  vector<double *>{para_pose_[imu_i].data(), para_pose_[imu_j].data(), para_Ex_Pose[0], para_Ex_Pose[1], para_feature_[feature_index].data(), para_Td[0]},
                  vector<int>{0, 4});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            }
            else
            {
              vins_mah::factor::ProjectionOneFrameTwoCam *f = new vins_mah::factor::ProjectionOneFrameTwoCam(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
              vins_mah::factor::ResidualBlockInfo *residual_block_info = new vins_mah::factor::ResidualBlockInfo(f, loss_function,
                  vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_feature_[feature_index].data(), para_Td[0]},
                  vector<int>{2});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            }
          }
        }
      }
    }

    marginalization_info->preMarginalize();
    marginalization_info->marginalize();

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= vio_options_.window_size; i++) {
      addr_shift[reinterpret_cast<long>(para_pose_[i].data())] = para_pose_[i - 1].data();
      if (vio_options_.use_imu) {
        addr_shift[reinterpret_cast<long>(para_speed_bias_[i].data())] = para_speed_bias_[i - 1].data();
      }
    }

    for (int i = 0; i < vio_options_.num_cameras; i++) {
      addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
    }

    addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

    std::vector<double *> parameter_blocks =
        marginalization_info->getParameterBlocks(addr_shift);

    if (last_marginalization_info)
        delete last_marginalization_info;

    last_marginalization_info = marginalization_info;
    last_marginalization_parameter_blocks = parameter_blocks;

  } else {

    const bool has_last_marginalization = last_marginalization_info != nullptr;

    const bool contains_pose =
        std::count(
            std::begin(last_marginalization_parameter_blocks),
            std::end(last_marginalization_parameter_blocks),
            para_pose_[vio_options_.window_size - 1].data()) > 0;

    if (has_last_marginalization && contains_pose)
    {
      vins_mah::factor::MarginalizationInfo *marginalization_info = new vins_mah::factor::MarginalizationInfo();

      vector2double();
      if (last_marginalization_info && last_marginalization_info->valid)
      {
        std::vector<int> drop_set;
        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
        {
          CHECK(last_marginalization_parameter_blocks[i] != para_speed_bias_[vio_options_.window_size - 1].data());
          if (last_marginalization_parameter_blocks[i] == para_pose_[vio_options_.window_size - 1].data())
            drop_set.push_back(i);
        }
        // construct new marginlization_factor
        vins_mah::factor::Marginalization *marginalization_factor = new vins_mah::factor::Marginalization(last_marginalization_info);
        vins_mah::factor::ResidualBlockInfo *residual_block_info = new vins_mah::factor::ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);

        marginalization_info->addResidualBlockInfo(residual_block_info);
      }

      marginalization_info->preMarginalize();

      marginalization_info->marginalize();

      std::unordered_map<long, double *> addr_shift;
      for (int i = 0; i <= vio_options_.window_size; i++) {
        if (i == vio_options_.window_size - 1)
          continue;
        else if (i == vio_options_.window_size)
        {
          addr_shift[reinterpret_cast<long>(para_pose_[i].data())] = para_pose_[i - 1].data();
          if(vio_options_.use_imu)
            addr_shift[reinterpret_cast<long>(para_speed_bias_[i].data())] = para_speed_bias_[i - 1].data();
        }
        else
        {
          addr_shift[reinterpret_cast<long>(para_pose_[i].data())] = para_pose_[i].data();
          if(vio_options_.use_imu)
            addr_shift[reinterpret_cast<long>(para_speed_bias_[i].data())] = para_speed_bias_[i].data();
        }
      }
      for (int i = 0; i < vio_options_.num_cameras; i++)
        addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

      addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];


      vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
      if (last_marginalization_info)
        delete last_marginalization_info;
      last_marginalization_info = marginalization_info;
      last_marginalization_parameter_blocks = parameter_blocks;

    }
  }

  if (vio_options_.increase_verbosity) {
    RCLCPP_INFO(node_->get_logger(), "Marginalization time: %.3f ms", marginalization_timer.toc());

    RCLCPP_INFO(node_->get_logger(), "Optimization time: %.3f ms", optimization_timer.toc());
  }
}

void Estimator::slideWindow() {
  vins_mah::utils::TicToc t_margin;
  if (marginalization_strategy_ == MarginalizationStrategy::REMOVE_OLDEST) {
    double t_0 = headers_[0];
    back_R0 = rs_[0];
    back_P0 = ps_[0];
    if (frame_count == vio_options_.window_size) {
      for (int i = 0; i < vio_options_.window_size; i++) {
        headers_[i] = headers_[i + 1];
        rs_[i].swap(rs_[i + 1]);
        ps_[i].swap(ps_[i + 1]);
        if (vio_options_.use_imu) {
          std::swap(pre_integrations_[i], pre_integrations_[i + 1]);

          dt_buf_[i].swap(dt_buf_[i + 1]);
          linear_acceleration_buf_[i].swap(linear_acceleration_buf_[i + 1]);
          angular_velocity_buf_[i].swap(angular_velocity_buf_[i + 1]);

          vs_[i].swap(vs_[i + 1]);
          bas_[i].swap(bas_[i + 1]);
          bgs_[i].swap(bgs_[i + 1]);
        }
      }
      headers_[vio_options_.window_size] = headers_[vio_options_.window_size - 1];
      ps_[vio_options_.window_size] = ps_[vio_options_.window_size - 1];
      rs_[vio_options_.window_size] = rs_[vio_options_.window_size - 1];

      if (vio_options_.use_imu) {
        vs_[vio_options_.window_size] = vs_[vio_options_.window_size - 1];
        bas_[vio_options_.window_size] = bas_[vio_options_.window_size - 1];
        bgs_[vio_options_.window_size] = bgs_[vio_options_.window_size - 1];

        pre_integrations_[vio_options_.window_size].reset();
        pre_integrations_[vio_options_.window_size] = std::make_unique<vins_mah::factor::IntegrationBase>(acc_0, gyr_0, bas_[vio_options_.window_size], bgs_[vio_options_.window_size], vio_options_.imu);

        dt_buf_[vio_options_.window_size].clear();
        linear_acceleration_buf_[vio_options_.window_size].clear();
        angular_velocity_buf_[vio_options_.window_size].clear();
      }

      if (true || (state_ == EstimatorState::INITIALIZATION)) { // ??
        std::map<double, vins_mah::initial::ImageFrame>::iterator it_0;
        it_0 = all_image_frame.find(t_0);
        it_0->second.pre_integration.reset();
        all_image_frame.erase(all_image_frame.begin(), it_0);
      }
      slideWindowOld();
    }
  } else {
    if (frame_count == vio_options_.window_size) {
      headers_[frame_count - 1] = headers_[frame_count];
      ps_[frame_count - 1] = ps_[frame_count];
      rs_[frame_count - 1] = rs_[frame_count];

      if (vio_options_.use_imu) {
        for (unsigned int i = 0; i < dt_buf_[frame_count].size(); i++) {
          double tmp_dt = dt_buf_[frame_count][i];
          Vector3d tmp_linear_acceleration = linear_acceleration_buf_[frame_count][i];
          Vector3d tmp_angular_velocity = angular_velocity_buf_[frame_count][i];

          pre_integrations_[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

          dt_buf_[frame_count - 1].push_back(tmp_dt);
          linear_acceleration_buf_[frame_count - 1].push_back(tmp_linear_acceleration);
          angular_velocity_buf_[frame_count - 1].push_back(tmp_angular_velocity);
        }

        vs_[frame_count - 1] = vs_[frame_count];
        bas_[frame_count - 1] = bas_[frame_count];
        bgs_[frame_count - 1] = bgs_[frame_count];

        pre_integrations_[vio_options_.window_size].reset();
        pre_integrations_[vio_options_.window_size] = std::make_unique<vins_mah::factor::IntegrationBase>(acc_0, gyr_0, bas_[vio_options_.window_size], bgs_[vio_options_.window_size], vio_options_.imu);

        dt_buf_[vio_options_.window_size].clear();
        linear_acceleration_buf_[vio_options_.window_size].clear();
        angular_velocity_buf_[vio_options_.window_size].clear();
      }
      slideWindowNew();
    }
  }
}

void Estimator::slideWindowNew() {
  sum_of_front++;
  feature_manager_.removeFront(frame_count);
}

void Estimator::slideWindowOld() {
  sum_of_back++;

  bool shift_depth = (state_ == EstimatorState::OPTIMIZATION) ? true : false;

  if (shift_depth) {
    Matrix3d R0, R1;
    Vector3d P0, P1;
    R0 = back_R0 * ric[0];
    R1 = rs_[0] * ric[0];
    P0 = back_P0 + back_R0 * tic[0];
    P1 = ps_[0] + rs_[0] * tic[0];
    feature_manager_.removeBackShiftDepth(R0, P0, R1, P1);
  } else {
    feature_manager_.removeBack();
  }
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = rs_[frame_count];
    T.block<3, 1>(0, 3) = ps_[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = rs_[index];
    T.block<3, 1>(0, 3) = ps_[index];
}

void Estimator::predictPtsInNextFrame() {
    //printf("predict pts in next frame\n");
    if (frame_count < 2) {
        return;
    }
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    std::map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : feature_manager_.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = rs_[firstIndex] * pts_j + ps_[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    feature_tracker_->setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici, Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, double depth, Vector3d &uvi, Vector3d &uvj) {
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : feature_manager_.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(rs_[imu_i], ps_[imu_i], ric[0], tic[0],
                                                    rs_[imu_j], ps_[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(vio_options_.use_stereo && it_per_frame.is_stereo)
            {

                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    double tmp_error = reprojectionError(rs_[imu_i], ps_[imu_i], ric[0], tic[0], 
                                                        rs_[imu_j], ps_[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(rs_[imu_i], ps_[imu_i], ric[0], tic[0], 
                                                        rs_[imu_j], ps_[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        double ave_err = err / errCnt;
        if (ave_err * vio_options_.cameras[0].camera_model->K.at<double>(0, 0) > 3) {
          removeIndex.insert(it_per_id.feature_id);
        }
    }
}

/* fastPredictIMU() //{ */

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity) {
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - estimated_gravity_;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * vins_mah::utils::Rotations::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - estimated_gravity_;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

//}

/* updateLatestStates() //{ */

void Estimator::updateLatestStates() {
    mPropagate.lock();
    latest_time = headers_[frame_count] + vio_options_.cam_time_offset;
    latest_P = ps_[frame_count];
    latest_Q = rs_[frame_count];
    latest_V = vs_[frame_count];
    latest_Ba = bas_[frame_count];
    latest_Bg = bgs_[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while(!tmp_accBuf.empty()) {
      double t = tmp_accBuf.front().first;
      Eigen::Vector3d acc = tmp_accBuf.front().second;
      Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
      fastPredictIMU(t, acc, gyr);
      tmp_accBuf.pop();
      tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}

//}

/* outliersRejection() //{ */

// added by myself
void Estimator::outliersRejection(set<int> &keepIndex, set<int> &removeIndex) {
  for(auto &it_per_id : feature_manager_.feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4)
      continue;

    int imu_i = it_per_id.start_frame;
    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    double depth = it_per_id.estimated_depth;

    pair<int, int> bestFrame = computeMaxParallaxFrame(it_per_id, frame_count);
    int imu_j = bestFrame.first;
    int camera = bestFrame.second;

    double mah2 = 0.0;

    if(imu_j != -1) {
      if(camera == 0) {
        Vector3d pts_j = it_per_id.feature_per_frame[imu_j - imu_i].point;
        Eigen::Matrix2d feature_cov = it_per_id.feature_per_frame[imu_j - imu_i].featureCovs[0];                
        mah2 = computeMahDist(rs_[imu_i], ps_[imu_i], ric[0], tic[0], 
            rs_[imu_j], ps_[imu_j], ric[0], tic[0],
            depth, pts_i, pts_j, feature_cov, imu_i, imu_j);
      } else {
        Vector3d pts_j = it_per_id.feature_per_frame[imu_j - imu_i].pointRight;
        Eigen::Matrix2d feature_cov = it_per_id.feature_per_frame[imu_j - imu_i].featureCovs[1];
        mah2 = computeMahDist(rs_[imu_i], ps_[imu_i], ric[0], tic[0], 
            rs_[imu_j], ps_[imu_j], ric[1], tic[1],
            depth, pts_i, pts_j, feature_cov, imu_i, imu_j);
      }
    }
    if (mah2 > vio_options_.outlier_chi_square_threshold) {
      removeIndex.insert(it_per_id.feature_id);
    } else {
      keepIndex.insert(it_per_id.feature_id);
    }
  }
}

//}

/* computeMahDist() //{ */

double Estimator::computeMahDist(const Matrix3d &Ri, const Vector3d &Pi,
                    const Matrix3d &rici, const Vector3d &tici,
                    const Matrix3d &Rj, const Vector3d &Pj,
                    const Matrix3d &ricj, const  Vector3d &ticj, 
                    const double depth, const Vector3d &uvi,
                    const Vector3d &uvj, const Matrix2d &feature_cov,
                    const int start_frame, const int cur_frame)
{
    // compute the complete Jacobian
    pair<Eigen::Vector2d, Eigen::Matrix<double, 2, 12>> temp = 
        computeJacobianAndResidual(Ri, Pi, rici, tici, Rj, Pj, ricj, ticj, depth, uvi, uvj);
    
    Eigen::Vector2d residual = temp.first;
    Eigen::Matrix<double, 2, 12> J = temp.second;

    // construct the state covariance matrix
    Eigen::Matrix<double,12,12> C = Eigen::Matrix<double,12,12>::Zero();
    C.block<6,6>(0,0) = pose_covs_[start_frame];
    C.block<6,6>(6,6) = pose_covs_[cur_frame];

    // tracking covariance plus reprojection covariance
    Eigen::Matrix2d C_E = feature_cov + J * C * J.transpose();

    // compute the square of Mahalanobis distance
    double mah2 = residual.transpose() * C_E.ldlt().solve(residual);

    return mah2;
}

//}

// computeJacobianAndResidual() //{ */

std::pair<Eigen::Vector2d, Eigen::Matrix<double, 2, 12>> Estimator::computeJacobianAndResidual(
    const Matrix3d &Ri,
    const Vector3d &Pi, 
    const Matrix3d &rici, const Vector3d &tici,
    const Matrix3d &Rj, const Vector3d &Pj,
    const Matrix3d &ricj, const Vector3d &ticj,
    const double depth, const Vector3d &uvi,
    const Vector3d &uvj) {

    // transform feature position from camera i frame into camera j frame
    Vector3d pts_ci = depth * uvi;
    Vector3d pts_w = Ri * (rici * pts_ci + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);

    // compute the residual
    double xj = pts_cj.x(), yj = pts_cj.y(), zj = pts_cj.z();
    Eigen::Vector2d residual = pts_cj.head<2>()/zj - uvj.head<2>();

    Eigen::Matrix<double, 2, 3> reduce;
    reduce << 1/zj, 0.0, -xj/(zj*zj),
    0.0, 1/zj, -yj/(zj*zj);

    // common block
    Eigen::Matrix<double, 2, 3> Rj_to_World =  reduce * ricj.transpose() * Rj.transpose();       

    // Jacobians
    Eigen::Matrix<double, 2, 3> J_bi = Rj_to_World;
    Eigen::Matrix<double, 2, 3> J_thetai = - Rj_to_World * Ri.transpose() * vins_mah::utils::Rotations::skewSymmetric(pts_ci);
    Eigen::Matrix<double, 2, 3> J_bj = - Rj_to_World;
    Eigen::Matrix<double, 2, 3> J_thetaj = - Rj_to_World * vins_mah::utils::Rotations::skewSymmetric(pts_w - Pj);

    // assemble Jacobians
    Eigen::Matrix<double, 2, 12> J;
    J << J_bi, J_thetai, J_bj, J_thetaj;

    return make_pair(residual, J);
}

//}

/* computeMaxParallaxFrame() //{ */

std::pair<int, int> Estimator::computeMaxParallaxFrame(const FeaturePerId& feature, const int cur_frame) {
    // compute the feature position relative to the world frame
    int imu_i = feature.start_frame;
    int imu_j = imu_i - 1;
    double depth = feature.estimated_depth;
    Vector3d pts_ci = depth * feature.feature_per_frame[0].point;
    Vector3d pts_w = rs_[imu_i] * (ric[0] * pts_ci + tic[0]) + ps_[imu_i];

    Eigen::Vector3d L1 = rs_[imu_i] * tic[0] + ps_[imu_i] - pts_w;

    int best_frame = -1;
    int best_camera = -1;
    double max_angle = 0.0;

    for (int i = 0; i < feature.used_num; i++) {
        imu_j++;
        if(imu_j != imu_i && imu_j != cur_frame && imu_j != imu_i + 1)
        {  
            Eigen::Vector3d L2 = rs_[imu_j] * tic[0] + ps_[imu_j] - pts_w;
            double cos_theta = L1.dot(L2) / ( L1.norm() * L2.norm() );
            double clipped = cos_theta < -1.0 ? -1.0
                  : (cos_theta >  1.0 ?  1.0 : cos_theta);
            double angle = std::acos(clipped); 
            if(angle > max_angle) {
                max_angle = angle;
                best_frame = imu_j;
                best_camera = 0;
            }
        }
        if(vio_options_.use_stereo && imu_j != imu_i && imu_j != cur_frame)
        {  
            Eigen::Vector3d L2= rs_[imu_j] * tic[1] + ps_[imu_j] - pts_w;
            double cos_theta = L1.dot(L2) / ( L1.norm() * L2.norm() );
            double clipped = cos_theta < -1.0 ? -1.0
                  : (cos_theta >  1.0 ?  1.0 : cos_theta);
            double angle = std::acos(clipped); 
            if(angle > max_angle) {
                max_angle = angle;
                best_frame = imu_j;
                best_camera = 1;
            }            
        }
    }

    return {best_frame, best_camera};
}

//}

/* computeHivnPP() //{ */

Eigen::MatrixXd Estimator::computeHinvPP(const MatrixXd& H_total) {
    const int m = vio_options_.window_size * 6;
    const int n = H_total.rows();
    assert(H_total.cols() == n);

    // RHS: [I; 0]
    MatrixXd E = MatrixXd::Zero(n, m);
    E.topLeftCorner(m, m).setIdentity();

    // 1) rank-revealing solve (min-norm)
    CompleteOrthogonalDecomposition<MatrixXd> cod(H_total);
    MatrixXd W0 = cod.solve(E);          // = H^† E

    // 2) project RHS without eigendecomp: E_proj = H * W0 = P_R(H) * E
    MatrixXd E_proj = H_total * W0;

    // 3) solve again on the projected RHS
    MatrixXd W = cod.solve(E_proj);
    MatrixXd X = W.topRows(m);

    // ---- verification (optional) ----
    // MatrixXd residual = H_total * W - E_proj;
    // double abs_res = residual.norm();
    // double rel_res = abs_res / E_proj.norm();

    // std::cout << "[computeX] rank(H) = " << cod.rank() << "/" << n
    //           << "  abs_res = " << abs_res
    //           << "  rel_res = " << rel_res << "\n";

    return X;
}

//}

// Visualization functions

/* publishOdometry() //{ */

void Estimator::publishOdometry(const rclcpp::Time &timestamp) {
  if (state_ != EstimatorState::OPTIMIZATION) 
    return;

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = timestamp;
  odometry.header.frame_id = node_namespace_ + "/vm_global";
  odometry.child_frame_id = node_namespace_ + "/vm_imu";
  Eigen::Quaterniond tmp_Q;
  tmp_Q = Quaterniond(rs_[vio_options_.window_size]);
  odometry.pose.pose.position.x = ps_[vio_options_.window_size].x();
  odometry.pose.pose.position.y = ps_[vio_options_.window_size].y();
  odometry.pose.pose.position.z = ps_[vio_options_.window_size].z();
  odometry.pose.pose.orientation.x = tmp_Q.x();
  odometry.pose.pose.orientation.y = tmp_Q.y();
  odometry.pose.pose.orientation.z = tmp_Q.z();
  odometry.pose.pose.orientation.w = tmp_Q.w();
  odometry.twist.twist.linear.x = vs_[vio_options_.window_size].x();
  odometry.twist.twist.linear.y = vs_[vio_options_.window_size].y();
  odometry.twist.twist.linear.z = vs_[vio_options_.window_size].z();

  publisher_odometry_->publish(odometry);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = timestamp;
  pose_stamped.header.frame_id = node_namespace_ + "/vm_global";
  pose_stamped.pose = odometry.pose.pose;

  odometry_path_.header.stamp = timestamp;
  odometry_path_.header.frame_id = node_namespace_ + "/vm_global";
  odometry_path_.poses.push_back(pose_stamped);

  publisher_path_->publish(odometry_path_);

  Eigen::Vector3d tmp_T = ps_[vio_options_.window_size];

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Time: %f, T: %.3f %.3f %.3f, Q: %.3f %.3f %.3f %.3f", timestamp.seconds(), tmp_T.x(), tmp_T.y(), tmp_T.z(), tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
}

//}

/* publishTrackImages() //{ */

void Estimator::publishTrackImages(const cv::Mat &track_img, const rclcpp::Time &timestamp) {
  std_msgs::msg::Header header;
  header.stamp = timestamp;
  header.frame_id = node_namespace_ + "/vm_cam0";
  auto out_msg = cv_bridge::CvImage(header, "bgr8", track_img).toImageMsg();

  it_publisher_track_.publish(out_msg);
}

//}

/* publishTfs() //{ */

void Estimator::publishTfs(const rclcpp::Time &timestamp) {
  if (state_ != EstimatorState::OPTIMIZATION) 
    return;

  Eigen::Isometry3d T_global2imu = Eigen::Isometry3d::Identity();
  T_global2imu.linear() = rs_[vio_options_.window_size];
  T_global2imu.translation() = ps_[vio_options_.window_size];

  // Invert transformation
  Eigen::Isometry3d T_imu2global = T_global2imu.inverse();

  // T = rotation and translation
  // R = rotation
  // t = translation
  const Eigen::Quaterniond R_imu2global(T_imu2global.rotation());
  const Eigen::Vector3d t_imu2global = T_imu2global.translation();

  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = timestamp;
  transform.header.frame_id = node_namespace_ + "/vm_imu";
  transform.child_frame_id = node_namespace_ + "/vm_global";

  transform.transform.translation.x = t_imu2global.x();
  transform.transform.translation.y = t_imu2global.y();
  transform.transform.translation.z = t_imu2global.z();

  transform.transform.rotation.x = R_imu2global.x();
  transform.transform.rotation.y = R_imu2global.y();
  transform.transform.rotation.z = R_imu2global.z();
  transform.transform.rotation.w = R_imu2global.w();

  transform_broadcaster_->sendTransform(transform);

  geometry_msgs::msg::TransformStamped tf_cam;

  tf_cam.header.stamp = timestamp;
  tf_cam.header.frame_id = node_namespace_ + "/vm_imu";
  tf_cam.child_frame_id = node_namespace_ + "/vm_cam0";

  tf_cam.transform.translation.x = tic[0].x();
  tf_cam.transform.translation.y = tic[0].y();
  tf_cam.transform.translation.z = tic[0].z();

  Eigen::Quaterniond cam_q(ric[0]);

  tf_cam.transform.rotation.x = cam_q.x();
  tf_cam.transform.rotation.y = cam_q.y();
  tf_cam.transform.rotation.z = cam_q.z();
  tf_cam.transform.rotation.w = cam_q.w();

  transform_broadcaster_->sendTransform(tf_cam);
}

//}

/* publishKeyPoses() //{ */

void Estimator::publishKeyPoses(const rclcpp::Time &timestamp) {
  if (key_poses_.empty())
    return;

  visualization_msgs::msg::Marker msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = node_namespace_ + "/vm_global";
  msg.ns = "key_poses";
  msg.id = 0;
  msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  msg.action = visualization_msgs::msg::Marker::ADD;

  msg.pose.orientation.w = 1.0;

  msg.scale.x = 0.05;
  msg.scale.y = 0.05;
  msg.scale.z = 0.05;

  msg.color.r = 1.0f;
  msg.color.g = 0.0f;
  msg.color.b = 0.0f;
  msg.color.a = 1.0f;

  // Infinite lifetime (default constructed Duration)
  msg.lifetime = builtin_interfaces::msg::Duration();

  for (int i = 0; i <= vio_options_.window_size; ++i) {
    geometry_msgs::msg::Point pose_marker;

    const Eigen::Vector3d& pose = key_poses_[i];

    pose_marker.x = pose.x();
    pose_marker.y = pose.y();
    pose_marker.z = pose.z();

    msg.points.push_back(pose_marker);
  }

  publisher_key_poses_->publish(msg);
}

//}

/* publishLandmarks() //{ */

void Estimator::publishLandmarks(const rclcpp::Time &timestamp) {

  // Publish active landmarks
  pcl::PointCloud<pcl::PointXYZ> point_cloud;

  for (auto &it_per_id : feature_manager_.feature) {
    int used_num = it_per_id.feature_per_frame.size();

    if (!(used_num >= 2 && it_per_id.start_frame < vio_options_.window_size - 2))
      continue;

    if (it_per_id.start_frame > vio_options_.window_size * 3.0 / 4.0 || it_per_id.solve_flag != 1)
      continue;

    int imu_i = it_per_id.start_frame;
    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
    Eigen::Vector3d w_pts_i = rs_[imu_i] * (ric[0] * pts_i + tic[0]) + ps_[imu_i];

    point_cloud.emplace_back(w_pts_i.x(), w_pts_i.y(), w_pts_i.z());
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(point_cloud, cloud_msg);
  cloud_msg.header.stamp = timestamp;
  cloud_msg.header.frame_id = node_namespace_ + "/vm_global";

  publisher_feature_landmarks_->publish(cloud_msg);


  // Publish marginalized landmarks
  pcl::PointCloud<pcl::PointXYZ> margin_cloud;

  for (auto &it_per_id : feature_manager_.feature) {
    int used_num = it_per_id.feature_per_frame.size();

    if (!(used_num >= 2 && it_per_id.start_frame < vio_options_.window_size - 2))
      continue;

    if (it_per_id.start_frame == 0 &&
        it_per_id.feature_per_frame.size() <= 2 &&
        it_per_id.solve_flag == 1) {
      int imu_i = it_per_id.start_frame;
      Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
      Eigen::Vector3d w_pts_i = rs_[imu_i] * (ric[0] * pts_i + tic[0]) + ps_[imu_i];

      margin_cloud.emplace_back(w_pts_i.x(), w_pts_i.y(), w_pts_i.z());
    }
  }

  sensor_msgs::msg::PointCloud2 margin_msg;
  pcl::toROSMsg(margin_cloud, margin_msg);
  margin_msg.header.stamp = timestamp;
  margin_msg.header.frame_id = node_namespace_ + "/vm_global";

  publisher_marginalized_landmarks_->publish(margin_msg);
}

//}

/* publishCameraVisualPoses() //{ */

void Estimator::publishCameraVisualPoses(const rclcpp::Time &timestamp) {
  if (state_ != EstimatorState::OPTIMIZATION)
    return;
  
  camera_pose_visualizer_->reset();

  int i = vio_options_.window_size - 1;
  Eigen::Vector3d P = ps_[i] + rs_[i] * tic[0];
  Eigen::Quaterniond R = Eigen::Quaterniond(rs_[i] * ric[0]);
  camera_pose_visualizer_->add_pose(P, R);

  if (vio_options_.use_stereo) {
    Eigen::Vector3d P = ps_[i] + rs_[i] * tic[1];
    Eigen::Quaterniond R = Eigen::Quaterniond(rs_[i] * ric[1]);
    camera_pose_visualizer_->add_pose(P, R);
  }

  auto camera_visual_poses = camera_pose_visualizer_->get_camera_visual_poses(timestamp, node_namespace_ + "/vm_global"); 
  publisher_camera_visual_poses_->publish(camera_visual_poses);
}

//}

/* printStatistics() //{ */

void Estimator::printStatistics(const double solver_time_ms) {
  if (state_ != EstimatorState::OPTIMIZATION)
    return;

  total_solver_time_ms_ += solver_time_ms;
  ++num_solver_runs_;

  RCLCPP_INFO(node_->get_logger(), "Average solver time: %.3f ms", total_solver_time_ms_ / num_solver_runs_);

  total_path_length_ += (ps_[vio_options_.window_size] - last_path_).norm();
  last_path_ = ps_[vio_options_.window_size];

  RCLCPP_INFO(node_->get_logger(), "Accumulated path: %.3f m", total_path_length_);

}

//}

} // namespace vins_mah
