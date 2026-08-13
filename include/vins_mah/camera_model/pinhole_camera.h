#ifndef VINS_MAH_CAMERA_MODEL_PINHOLE_CAMERA_H_
#define VINS_MAH_CAMERA_MODEL_PINHOLE_CAMERA_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace vins_mah
{
  namespace camera_model
  {

    class PinholeCamera
    {

      public:
        cv::Mat K;
        cv::Mat D;

        Eigen::Vector3d liftProjective(const cv::Point2f& p) const;
        Eigen::Vector2d spaceToPlane(const Eigen::Vector3d& P) const;
    };

  } // namespace camera_model

} // namespace vins_mah

#endif // VINS_MAH_CAMERA_MODEL_PINHOLE_CAMERA_H_
