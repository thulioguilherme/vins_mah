#include "vins_mah/camera_model/pinhole_camera.h"

namespace vins_mah
{
  namespace camera_model
  {

    Eigen::Vector2d PinholeCamera::spaceToPlane(const Eigen::Vector3d& P) const
    {
      cv::Mat objectPoints(1, 1, CV_64FC3);
      objectPoints.at<cv::Vec3d>(0, 0) = cv::Vec3d(P.x(), P.y(), P.z());

      cv::Mat imagePoints;

      cv::projectPoints(
          objectPoints,
          cv::Vec3d::zeros(),
          cv::Vec3d::zeros(),
          K,
          D,
          imagePoints);

      const cv::Vec2d& uv = imagePoints.at<cv::Vec2d>(0, 0);

      return Eigen::Vector2d(uv[0], uv[1]);
    }

    Eigen::Vector3d PinholeCamera::liftProjective(const cv::Point2f& p) const
    {
      cv::Mat imagePoints(1, 1, CV_64FC2);
      imagePoints.at<cv::Vec2d>(0, 0) = cv::Vec2d(
          static_cast<double>(p.x),
          static_cast<double>(p.y));

      cv::Mat normalizedPoints;

      cv::undistortPoints(
          imagePoints,
          normalizedPoints,
          K,
          D);

      const cv::Vec2d& xy = normalizedPoints.at<cv::Vec2d>(0, 0);

      return Eigen::Vector3d(
          xy[0],
          xy[1],
          1.0);
    }

  } // namespace camera_model

} // namespace vins_mah
