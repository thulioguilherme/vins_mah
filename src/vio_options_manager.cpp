#include "vins_mah/vio_options_manager.h"

namespace vins_mah
{

/* VIO options manager constructor //{ */

VioOptionsManager::VioOptionsManager(const rclcpp::Node::SharedPtr &node) : node_(node) {
  RCLCPP_INFO(node_->get_logger(), "VIO Options manager created!");
}

//}

/* getOptions() //{ */

vins_mah::VioOptions VioOptionsManager::getOptions() {

  // Load parameters using ROS
  getVioOptions();

  // Load the remaining parameters using Yaml
  std::string package_path = ament_index_cpp::get_package_share_directory("vins_mah");
  std::filesystem::path cfg_folder = std::filesystem::path(package_path) / "config" / vio_options_.cfg_folder;

  // Get IMU configuration path and load it
  std::filesystem::path cfg_imu_path = cfg_folder / vio_options_.cfg_imu_file;
  YAML::Node cfg_imu_yaml = YAML::LoadFile(cfg_imu_path.string());
  getImuOptions(cfg_imu_yaml, "imu0");
  RCLCPP_INFO(node_->get_logger(), "IMU configuration file: %s", cfg_imu_path.string().c_str());

  // Get IMU-camera configuration path and load it
  std::filesystem::path cfg_imucam_path = cfg_folder / vio_options_.cfg_imucam_file;
  YAML::Node cfg_imucam_yaml = YAML::LoadFile(cfg_imucam_path.string());
  getImuCameraOptions(cfg_imucam_yaml);
  RCLCPP_INFO(node_->get_logger(), "IMU-camera configuration file: %s", cfg_imucam_path.string().c_str());

  // Setup mask image
  if (vio_options_.use_mask) {
    std::filesystem::path mask_path = cfg_folder / vio_options_.mask_file;
    vio_options_.mask_img = cv::imread(mask_path.string(), cv::IMREAD_GRAYSCALE);

    if (vio_options_.mask_img.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load mask image: %s. We will disable the USE_MASK option!", mask_path.string().c_str());
      vio_options_.use_mask = false;
      vio_options_.mask_img = cv::Mat(vio_options_.cameras[0].height, vio_options_.cameras[0].width, CV_8UC1, cv::Scalar(255));
    }
  } else {
    vio_options_.mask_img = cv::Mat(vio_options_.cameras[0].height, vio_options_.cameras[0].width, CV_8UC1, cv::Scalar(255));
  }

  //
  vio_options_.min_parallax = vio_options_.min_parallax / vio_options_.cameras[0].camera_model->K.at<double>(0, 0);
  RCLCPP_INFO(node_->get_logger(), "Min. parallax: %lf", vio_options_.min_parallax);

  return vio_options_;
}

//}

/* getVioOptions() //{ */

void VioOptionsManager::getVioOptions() {

  // Get configuration folder path
  node_->declare_parameter<std::string>("config_folder", "viode");
  node_->get_parameter("config_folder", vio_options_.cfg_folder);

  node_->declare_parameter<std::string>("config_imu", "imu.yaml");
  node_->get_parameter("config_imu", vio_options_.cfg_imu_file);

  node_->declare_parameter<std::string>("config_imucam", "imucam.yaml");
  node_->get_parameter("config_imucam", vio_options_.cfg_imucam_file);

  node_->declare_parameter<int>("window_size", 20);
  node_->get_parameter("window_size", vio_options_.window_size);

  node_->declare_parameter<double>("outlier_chi_square_threshold", 5.991);
  node_->get_parameter("outlier_chi_square_threshold", vio_options_.outlier_chi_square_threshold);

  node_->declare_parameter<double>("image_sync_timestamp_tolerance", 0.003);
  node_->get_parameter("image_sync_timestamp_tolerance", vio_options_.image_sync_timestamp_tolerance);

  node_->declare_parameter<bool>("use_reliable_qos", true);
  node_->get_parameter("use_reliable_qos", vio_options_.use_reliable_qos);

  node_->declare_parameter<bool>("increase_verbosity", false);
  node_->get_parameter("increase_verbosity", vio_options_.increase_verbosity);

  node_->declare_parameter<bool>("use_stereo", true);
  node_->get_parameter("use_stereo", vio_options_.use_stereo);

  node_->declare_parameter<bool>("use_imu", true);
  node_->get_parameter("use_imu", vio_options_.use_imu);

  node_->declare_parameter<bool>("use_multithreading", true);
  node_->get_parameter("use_multithreading", vio_options_.use_multithreading);

  node_->declare_parameter<bool>("use_mask", false);
  node_->get_parameter("use_mask", vio_options_.use_mask);

  // NOTE: we expect the mask file to be on the config folder
  node_->declare_parameter<std::string>("mask_file", "mask.png");
  node_->get_parameter("mask_file", vio_options_.mask_file);

  node_->declare_parameter<bool>("use_flow_back", true);
  node_->get_parameter("use_flow_back", vio_options_.use_flow_back);

  node_->declare_parameter<bool>("publish_track_images", true);
  node_->get_parameter("publish_track_images", vio_options_.publish_track_images);

  // Feature tracker paratemers
  node_->declare_parameter<int>("num_pts", 180);
  node_->get_parameter("num_pts", vio_options_.num_pts);

  node_->declare_parameter<int>("min_pixel_dist", 20);
  node_->get_parameter("min_pixel_dist", vio_options_.min_pixel_dist);

  node_->declare_parameter<double>("ransac_reproj_threshold", 1.0);
  node_->get_parameter("ransac_reproj_threshold", vio_options_.ransac_reproj_threshold);

  // Calibration parameters
  node_->declare_parameter<bool>("calib_cam_time_offset", false);
  node_->get_parameter("calib_cam_time_offset", vio_options_.calib_cam_time_offset);

  node_->declare_parameter<double>("cam_time_offset", 0.0);
  node_->get_parameter("cam_time_offset", vio_options_.cam_time_offset);

  std::string calib_cam_extrinsics_mode;
  node_->declare_parameter<std::string>("calib_cam_extrinsics_mode", "fixed");
  node_->get_parameter("calib_cam_extrinsics_mode", calib_cam_extrinsics_mode);

  if (calib_cam_extrinsics_mode == "fixed") {
    vio_options_.calib_cam_extrinsics_mode = ExtrinsicEstimationMode::FIXED;
  } else if (calib_cam_extrinsics_mode == "optimize") {
    vio_options_.calib_cam_extrinsics_mode = ExtrinsicEstimationMode::OPTIMIZE;
  } else {
    vio_options_.calib_cam_extrinsics_mode = ExtrinsicEstimationMode::CALIBRATE;
  }

  // Optimization parameters
  node_->declare_parameter<double>("solver_max_time", 0.2);
  node_->get_parameter("solver_max_time", vio_options_.solver_max_time);

  node_->declare_parameter<int>("solver_max_num_iterations", 8);
  node_->get_parameter("solver_max_num_iterations", vio_options_.solver_max_num_iterations);

  //
  node_->declare_parameter<double>("init_depth", 5.01);
  node_->get_parameter("init_depth", vio_options_.init_depth);

  node_->declare_parameter<double>("min_parallax", 10.0);
  node_->get_parameter("min_parallax", vio_options_.min_parallax);
}

//}

/* getImuOptions() //{ */

void VioOptionsManager::getImuOptions(const YAML::Node &root, const std::string &imu_name) {
  const auto cfg_imu = root[imu_name];

  vio_options_.imu.accelerometer_noise = cfg_imu["accelerometer_noise"].as<double>();
  vio_options_.imu.accelerometer_random_walk = cfg_imu["accelerometer_random_walk"].as<double>();
  vio_options_.imu.accelerometer_bias_threshold = cfg_imu["accelerometer_bias_threshold"].as<double>();

  vio_options_.imu.gyroscope_noise = cfg_imu["gyroscope_noise"].as<double>();
  vio_options_.imu.gyroscope_random_walk = cfg_imu["gyroscope_random_walk"].as<double>();
  vio_options_.imu.gyroscope_bias_threshold = cfg_imu["gyroscope_bias_threshold"].as<double>();

  double gravity_magnitude = cfg_imu["gravity_magnitude"].as<double>();
  Eigen::Vector3d gravity_reference{0.0, 0.0, gravity_magnitude};
  vio_options_.imu.gravity_reference = gravity_reference;
}

//}

/* getCameraOptions() //{ */

vins_mah::CameraOptions VioOptionsManager::getCameraOptions(const YAML::Node &root, const std::string &camera_name) {
  const auto cfg_imucam = root[camera_name];
  vins_mah::CameraOptions cam_options;

  Eigen::Matrix4d T_imu2cam;
  for (int i = 0; i < 4; ++i) {
    auto row = cfg_imucam["T_imu_cam"][i].as<std::vector<double>>();
    for (int j = 0; j < 4; ++j) {
      T_imu2cam(i, j) = row[j];
    }
  }

  cam_options.T_ItoC = T_imu2cam.block<3,1>(0,3);
  cam_options.R_ItoC = T_imu2cam.block<3,3>(0,0);

  auto resolution = cfg_imucam["resolution"].as<std::vector<int>>();
  cam_options.width = resolution[0];
  cam_options.height = resolution[1];

  RCLCPP_INFO(
    node_->get_logger(),
    "Camera resolution %s: %d (height) x %d (width)",
    camera_name.c_str(),
    cam_options.height,
    cam_options.width
  );

  // get camera model
  auto pinhole_camera = std::make_shared<vins_mah::camera_model::PinholeCamera>();

  auto intrinsics = cfg_imucam["intrinsics"].as<std::vector<double>>();
  double fx = intrinsics[0];
  double fy = intrinsics[1];
  double cx = intrinsics[2];
  double cy = intrinsics[3];

  pinhole_camera->K = (cv::Mat_<double>(3, 3) <<
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0);

  auto distortion = cfg_imucam["distortion_coeffs"].as<std::vector<double>>();
  pinhole_camera->D = (cv::Mat_<double>(4, 1) <<
    distortion[0],
    distortion[1],
    distortion[2],
    distortion[3]);

  cam_options.camera_model = pinhole_camera;

  return cam_options;
}

//}

/* getImuCameraOptions() //{ */

void VioOptionsManager::getImuCameraOptions(const YAML::Node &root) {
  vins_mah::CameraOptions cam_options0 = getCameraOptions(root, "cam0");
  vio_options_.cameras.push_back(cam_options0);

  if (vio_options_.use_stereo) {
    vins_mah::CameraOptions cam_options1 = getCameraOptions(root, "cam1");
    vio_options_.cameras.push_back(cam_options1);
    vio_options_.num_cameras = 2;
  }
}

//}

} // namespace vins_mah
