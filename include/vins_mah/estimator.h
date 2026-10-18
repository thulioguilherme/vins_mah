/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef VINS_MAH_ESTIMATOR_H_
#define VINS_MAH_ESTIMATOR_H_

#include <thread>
#include <mutex>
#include <std_msgs/msg/header.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <std_msgs/msg/float64.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <Eigen/Sparse>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "vins_mah/vio_types.h"
#include "vins_mah/vio_options.h"

#include "vins_mah/utils/rotations.h"
#include "vins_mah/utils/tic_toc.h"
#include "vins_mah/utils/camera_pose_visualization.h"

#include "vins_mah/initial/solve_5pts.h"
#include "vins_mah/initial/sfm.h"
#include "vins_mah/initial/alignment.h"
#include "vins_mah/initial/ex_rotation.h"

#include "vins_mah/factor/imu.h"
#include "vins_mah/factor/pose_local_parameterization.h"
#include "vins_mah/factor/marginalization.h"
#include "vins_mah/factor/projection_two_frame_one_cam.h"
#include "vins_mah/factor/projection_two_frame_two_cam.h"
#include "vins_mah/factor/projection_one_frame_two_cam.h"

#include "vins_mah/feature_tracker.h"
#include "vins_mah/feature_manager.h"

// added by myself
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Eigenvalues>
#include <sstream>
#include <vector>
#include <iomanip>


namespace vins_mah {

class Estimator
{

public:
  // Constructor
  explicit Estimator(const rclcpp::Node::SharedPtr &node, const std::string node_namespace, const vins_mah::VioOptions &options);
  ~Estimator();

  void setParameters();

  rclcpp::Node::SharedPtr node_;
  std::string node_namespace_;
  vins_mah::VioOptions vio_options_;
  Eigen::Vector3d estimated_gravity_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  void publishTfs(const rclcpp::Time &timestamp);

  // interface
  void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
  void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
  void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 11, 1>>>> &featureFrame);
  void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
  void processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
  void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 11, 1>>>> &image, const double header);
  void processMeasurements();
  void changeSensorType(bool use_imu, bool use_stereo);

  // internal
  void clearState();
  bool initialStructure();
  bool visualInitialAlign();
  bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();
  void optimization();
  void vector2double();
  void double2vector();
  bool failureDetection();
  bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, vector<pair<double, Eigen::Vector3d>> &gyrVector);
  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
  void predictPtsInNextFrame();
  void outliersRejection(set<int> &removeIndex);
  double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici, Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, double depth, Vector3d &uvi, Vector3d &uvj);
  void updateLatestStates();
  void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
  bool IMUAvailable(double t);
  void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

  std::mutex process_mutex_;
  std::thread process_thread_;

  std::mutex mBuf;
  std::mutex mPropagate;
  std::queue<pair<double, Eigen::Vector3d>> accBuf;
  std::queue<pair<double, Eigen::Vector3d>> gyrBuf;
  std::queue<pair<double, std::map<int, vector<pair<int, Eigen::Matrix<double, 11, 1> > > > > > featureBuf;
  double prevTime, curTime;
  bool openExEstimation;

  std::thread trackThread;

  std::shared_ptr<vins_mah::FeatureTracker> feature_tracker_;

  vins_mah::EstimatorState state_;
  vins_mah::MarginalizationStrategy marginalization_strategy_;

  Eigen::Matrix3d ric[2];
  Eigen::Vector3d tic[2];

  std::vector<Eigen::Vector3d> ps_;
  std::vector<Eigen::Vector3d> vs_;
  std::vector<Eigen::Matrix3d> rs_;
  std::vector<Eigen::Vector3d> bas_;
  std::vector<Eigen::Vector3d> bgs_;

  std::vector<double> headers_;

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;

  Eigen::Vector3d acc_0, gyr_0;

  std::vector<std::unique_ptr<vins_mah::factor::IntegrationBase>> pre_integrations_;

  std::vector<std::vector<double>> dt_buf_;
  std::vector<std::vector<Eigen::Vector3d>> linear_acceleration_buf_;
  std::vector<std::vector<Eigen::Vector3d>> angular_velocity_buf_;

  int frame_count;
  int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
  int inputImageCnt;

  vins_mah::FeatureManager feature_manager_;
  vins_mah::initial::MotionEstimator m_estimator;
  std::shared_ptr<vins_mah::initial::ExRotation> initial_ex_rotation_;

  bool first_imu;
  bool is_valid, is_key;
  bool failure_occur;

  std::vector<Vector3d> point_cloud;
  std::vector<Vector3d> margin_cloud;
  std::vector<Vector3d> key_poses_;
  double initial_timestamp;

  std::vector<std::array<double, StateSize::POSE>> para_pose_;
  std::vector<std::array<double, StateSize::SPEED_BIAS>> para_speed_bias_;
  std::vector<std::array<double, StateSize::FEATURE>> para_feature_;
  double para_Ex_Pose[2][StateSize::POSE];
  double para_Retrive_Pose[StateSize::POSE];
  double para_Td[1][1];
  double para_Tr[1][1];

  int loop_window_index;

  vins_mah::factor::MarginalizationInfo *last_marginalization_info = nullptr;
  std::vector<double *> last_marginalization_parameter_blocks;

  std::map<double, vins_mah::initial::ImageFrame> all_image_frame;
  std::unique_ptr<vins_mah::factor::IntegrationBase> tmp_pre_integration;

  Eigen::Vector3d initP;
  Eigen::Matrix3d initR;

  double latest_time;
  Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
  Eigen::Quaterniond latest_Q;

  bool initFirstPoseFlag;
  bool thread_initialized_;

  //
  void printStatistics(const double solver_time_ms);
  double total_path_length_{0};
  Eigen::Vector3d last_path_;
  double total_solver_time_ms_{0.0};
  int num_solver_runs_{0};

  // Publishers
  void publishTrackImages(const cv::Mat &track_img, const rclcpp::Time &timestamp);
  image_transport::Publisher it_publisher_track_;

  void publishOdometry(const rclcpp::Time &timestamp);
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;
  nav_msgs::msg::Path odometry_path_;

  void publishKeyPoses(const rclcpp::Time &timestamp);
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_key_poses_;

  void publishLandmarks(const rclcpp::Time &timestamp);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_feature_landmarks_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_marginalized_landmarks_;

  void publishCameraVisualPoses(const rclcpp::Time &timestamp);
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_camera_visual_poses_;
  std::unique_ptr<vins_mah::utils::CameraPoseVisualization> camera_pose_visualizer_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_num_features_;

  // added by myself
  std::vector<Eigen::Matrix<double, StateSize::POSE - 1, StateSize::POSE - 1>> pose_covs_;

  void outliersRejection(set<int> &keepIndex, set<int> &removeIndex);
  double computeMahDist(const Matrix3d &Ri, const Vector3d &Pi,
      const Matrix3d &rici, const Vector3d &tici,
      const Matrix3d &Rj, const Vector3d &Pj,
      const Matrix3d &ricj, const  Vector3d &ticj,
      const double depth, const Vector3d &uvi,
      const Vector3d &uvj, const Matrix2d &feature_cov,
      const int start_frame, const int cur_frame);
  std::pair<Eigen::Vector2d, Eigen::Matrix<double, 2, 12>> computeJacobianAndResidual(const Matrix3d &Ri, const Vector3d &Pi,
      const Matrix3d &rici, const Vector3d &tici,
      const Matrix3d &Rj, const Vector3d &Pj,
      const Matrix3d &ricj, const Vector3d &ticj,
      const double depth, const Vector3d &uvi,
      const Vector3d &uvj);
  std::pair<int, int> computeMaxParallaxFrame(const FeaturePerId& feature, const int cur_frame);
  Eigen::MatrixXd computeHinvPP(const MatrixXd& H_total);
};

} // namespace vins_mah

#endif // VINS_MAH_ESTIMATOR_H_
