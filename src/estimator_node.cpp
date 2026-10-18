#include "vins_mah/estimator_node.h"

namespace vins_mah
{

/* Estimator node constructor //{ */

EstimatorNode::EstimatorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("estimator_node", options) {
  timer_preinitialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&EstimatorNode::callbackTimerPreinitialization, this));
}

//}

/* callbackTimerPreinitialization() //{ */

void EstimatorNode::callbackTimerPreinitialization() {
  node_ = this->shared_from_this();
  initialize();
  timer_preinitialization_->cancel();
}

//}

/* initialize() //{ */

void EstimatorNode::initialize() {
  RCLCPP_INFO(get_logger(), "Initializing node!");

  // Get node namespace
  node_->declare_parameter<std::string>("node_namespace", "uav1");
  node_->get_parameter("node_namespace", _node_namespace_);

  vio_options_manager_ = std::make_shared<vins_mah::VioOptionsManager>(shared_from_this());
  vio_options_ = vio_options_manager_->getOptions();

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  estimator_ = std::make_shared<Estimator>(shared_from_this(), _node_namespace_, vio_options_);
  estimator_->setParameters();

  rclcpp::QoS qos = vio_options_.use_reliable_qos ? rclcpp::QoS(1000).reliable() : rclcpp::SensorDataQoS();

  // subscribers
  camera0_image_subcriber_ = image_transport::create_subscription(
    this,
    "cam0_in",
    std::bind(&EstimatorNode::callbackCamera0Image, this, std::placeholders::_1),
    "raw",
    qos.get_rmw_qos_profile()
  );

  if (vio_options_.use_stereo) {
    camera1_image_subcriber_ = image_transport::create_subscription(
      this,
      "cam1_in",
      std::bind(&EstimatorNode::callbackCamera1Image, this, std::placeholders::_1),
      "raw",
      qos.get_rmw_qos_profile()
    );
  }

  if (vio_options_.use_imu) {
    imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>("imu_in", qos, std::bind(&EstimatorNode::callbackImu, this, std::placeholders::_1));
  }

  sync_thread_ = std::thread(&EstimatorNode::syncronizeProcess, this);

  RCLCPP_INFO(this->get_logger(), "Node initialized!");
  is_initialized_ = true;
}

//}

/* callbackCamera0Image() //{ */

void EstimatorNode::callbackCamera0Image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  std::lock_guard<std::mutex> lock(m_buf_);
  img0_buf_.push(msg);
}

//}

/* callbackCamera1Image() //{ */

void EstimatorNode::callbackCamera1Image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  std::lock_guard<std::mutex> lock(m_buf_);
  img1_buf_.push(msg);
}

//}

/* callbackImu() //{ */

void EstimatorNode::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  double t = rclcpp::Time(msg->header.stamp).seconds();

  double dx = msg->linear_acceleration.x;
  double dy = msg->linear_acceleration.y;
  double dz = msg->linear_acceleration.z;

  double rx = msg->angular_velocity.x;
  double ry = msg->angular_velocity.y;
  double rz = msg->angular_velocity.z;

  Eigen::Vector3d acc(dx, dy, dz);
  Eigen::Vector3d gyr(rx, ry, rz);

  estimator_->inputIMU(t, acc, gyr);
}

//}

/* extractImageFromROSMsg() //{ */

cv::Mat EstimatorNode::extractImageFromROSMsg(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (msg->encoding == "8UC1") {
    cv::Mat img(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);

    return img.clone();
  } else {
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    return ptr->image;
  }
}

//}

/* syncronizeProcess() //{ */

void EstimatorNode::syncronizeProcess() {
  while (rclcpp::ok()) {
    if (vio_options_.use_stereo) {
      cv::Mat image0, image1;
      std_msgs::msg::Header header;
      double time = 0;

      {
        std::lock_guard<std::mutex> lock(m_buf_);

        if (!img0_buf_.empty() && !img1_buf_.empty()) {
          double time0 = rclcpp::Time(img0_buf_.front()->header.stamp).seconds();
          double time1 = rclcpp::Time(img1_buf_.front()->header.stamp).seconds();

          // Sync tolerance
          if (time0 < (time1 - vio_options_.image_sync_timestamp_tolerance)) {
            img0_buf_.pop();
            double delta = time1 - time0;
            RCLCPP_WARN(this->get_logger(), "Throwing img0 (delta: %.5f)", delta);
          } else if (time0 > (time1 + vio_options_.image_sync_timestamp_tolerance)) {
            img1_buf_.pop();
            double delta = time0 - time1;
            RCLCPP_WARN(this->get_logger(), "Throwing img1 (delta: %.5f)", delta);
          } else {
            time = time0;

            header = img0_buf_.front()->header;

            image0 = extractImageFromROSMsg(img0_buf_.front());
            img0_buf_.pop();

            image1 = extractImageFromROSMsg(img1_buf_.front());
            img1_buf_.pop();
          }
        }
      }

      if (!image0.empty()) {
        estimator_->inputImage(time, image0, image1);
      }
    } else {
      cv::Mat image;
      std_msgs::msg::Header header;
      double time = 0;

      {
        std::lock_guard<std::mutex> lock(m_buf_);

        if (!img0_buf_.empty()) {
          time = rclcpp::Time(img0_buf_.front()->header.stamp).seconds();

          header = img0_buf_.front()->header;

          image = extractImageFromROSMsg(img0_buf_.front());

          img0_buf_.pop();
        }
      }

      if (!image.empty()) {
        estimator_->inputImage(time, image);
      }
    }

  }
}

//}

} // namespace vins_mah

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vins_mah::EstimatorNode)
