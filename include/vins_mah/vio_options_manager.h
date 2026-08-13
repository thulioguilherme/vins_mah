#ifndef VINS_MAH_VIO_OPTIONS_MANAGER_H_
#define VINS_MAH_VIO_OPTIONS_MANAGER_H_

#include <filesystem>
#include <string>
#include <map>

#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include "vio_options.h"

namespace vins_mah {

class VioOptionsManager
{

public:
  // Constructor
  VioOptionsManager(const rclcpp::Node::SharedPtr &node);
  vins_mah::VioOptions getOptions();

private:
  rclcpp::Node::SharedPtr node_;
  vins_mah::VioOptions vio_options_;
  vins_mah::CameraOptions getCameraOptions(const YAML::Node &root, const std::string &camera_name);

  void getVioOptions();
  void getImuOptions(const YAML::Node &root, const std::string &imu_name);
  void getImuCameraOptions(const YAML::Node &root);
};

} // namespace vins_mah

#endif // VINS_MAH_VIO_OPTIONS_MANAGER_H_
