#ifndef VINS_MAH_ESTIMATOR_NODE_H_
#define VINS_MAH_ESTIMATOR_NODE_H_

/* includes //{ */

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <glog/logging.h>

#include "vins_mah/vio_options.h"
#include "vins_mah/vio_options_manager.h"

#include "vins_mah/camera_model/pinhole_camera.h"

#include "vins_mah/estimator.h"

//}

namespace vins_mah
{

class EstimatorNode : public rclcpp::Node {

public:
  explicit EstimatorNode(const rclcpp::NodeOptions& options);

private:
  void initialize();

  rclcpp::TimerBase::SharedPtr timer_preinitialization_;
  void callbackTimerPreinitialization();

  std::string _uav_name_;

  std::atomic<bool> is_initialized_ = false;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<Estimator> estimator_;

  std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf_;
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img0_buf_;
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img1_buf_;

  std::mutex m_buf_;

  std::thread sync_thread_;
  void syncronizeProcess();

  // subscribers
  image_transport::Subscriber camera0_image_subcriber_;
  void callbackCamera0Image(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  image_transport::Subscriber camera1_image_subcriber_;
  void callbackCamera1Image(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  // auxiliary functions
  cv::Mat extractImageFromROSMsg(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  std::shared_ptr<vins_mah::VioOptionsManager> vio_options_manager_;
  vins_mah::VioOptions vio_options_;
};

} // namespace vins_mah

#endif // VINS_MAH_ESTIMATOR_NODE_H_
