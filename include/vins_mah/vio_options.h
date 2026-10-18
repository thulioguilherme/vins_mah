#ifndef VINS_MAH_VIO_OPTIONS_H_
#define VINS_MAH_VIO_OPTIONS_H_

#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "vins_mah/vio_types.h"
#include "vins_mah/camera_model/pinhole_camera.h"

namespace vins_mah {

struct ImuOptions {
  double accelerometer_noise;
  double accelerometer_random_walk;
  double accelerometer_bias_threshold;

  double gyroscope_noise;
  double gyroscope_random_walk;
  double gyroscope_bias_threshold;

  Eigen::Vector3d gravity_reference;
};

struct CameraOptions {

  std::string camera_name;
  std::shared_ptr<vins_mah::camera_model::PinholeCamera> camera_model;

  Eigen::Matrix3d R_ItoC;
  Eigen::Vector3d T_ItoC;

  int width, height;

  cv::Mat mask_image;
};

struct VioOptions {

  std::string cfg_folder;
  std::string cfg_imucam_file;
  std::string cfg_imu_file;

  // Uncertainty estimator parameters
  int window_size;
  double outlier_chi_square_threshold;
  double image_sync_timestamp_tolerance;

  double init_depth;
  double min_parallax;
  int num_cameras{1};
  bool use_stereo; // old name: stereo
  bool use_imu;
  bool use_multithreading; // old name: multiple_thread
  bool use_mask; // old name: masking
  std::string mask_file;
  cv::Mat mask_img; // old name: masking_path
  bool use_reliable_qos;

  // Optimization parameters
  double solver_max_time; // old name: max_solver_time
  int solver_max_num_iterations; // old name: max_num_iterations

  // Feature tracker parameters
  int num_pts; // old name: max_cnt
  int min_pixel_dist; // old name: min_dist
  double ransac_reproj_threshold; // old name: T_threshold
  bool publish_track_images;
  bool use_flow_back; // old name: flow_back

  // Calibration parameters
  vins_mah::ExtrinsicEstimationMode calib_cam_extrinsics_mode; // old name: estimate_extrinsic
  bool calib_cam_time_offset; // old name: estimate_td
  double cam_time_offset; // old name: td

  vins_mah::ImuOptions imu;
  std::vector<vins_mah::CameraOptions> cameras;

  std::string ex_calib_result_path;
  std::string vins_result_path;
  std::string output_folder;
  // pts_gt for debug purpose;
  std::map<int, Eigen::Vector3d> pts_gt;

  bool increase_verbosity{false};
};

} // namespace vins_mah

#endif // VINS_MAH_VIO_OPTIONS_H_
