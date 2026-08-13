/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef VINS_MAH_FACTOR_IMU_H_
#define VINS_MAH_FACTOR_IMU_H_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include <glog/logging.h>

#include "vins_mah/vio_types.h"
#include "vins_mah/utils/rotations.h"
#include "vins_mah/factor/integration_base.h"


namespace vins_mah
{
  namespace factor
  {

    class Imu : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
    {
      public:
        Imu() = delete;
        Imu(vins_mah::factor::IntegrationBase* _pre_integration, Eigen::Vector3d gravity_vector) : pre_integration(_pre_integration), gravity_vector_(gravity_vector)
      {
      }
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {

          Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
          Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

          Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
          Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
          Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

          Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
          Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

          Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
          Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
          Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

          //Eigen::Matrix<double, 15, 15> Fd;
          //Eigen::Matrix<double, 15, 12> Gd;

          //Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
          //Eigen::Quaterniond pQj = Qi * delta_q;
          //Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
          //Eigen::Vector3d pBaj = Bai;
          //Eigen::Vector3d pBgj = Bgi;

          //Vi + Qi * delta_v - g * sum_dt = Vj;
          //Qi * delta_q = Qj;

          //delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
          //delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
          //delta_q = Qi.inverse() * Qj;

#if 0
          if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
              (Bgi - pre_integration->linearized_bg).norm() > 0.01)
          {
            pre_integration->repropagate(Bai, Bgi);
          }
#endif

          Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
          residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);

          Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
          //sqrt_info.setIdentity();
          residual = sqrt_info * residual;

          if (jacobians)
          {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(vins_mah::StateOrder::POSITION, vins_mah::StateOrder::ACCELERATION_BIAS);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(vins_mah::StateOrder::POSITION, vins_mah::StateOrder::GYROSCOPE_BIAS);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::GYROSCOPE_BIAS);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(vins_mah::StateOrder::VELOCITY, vins_mah::StateOrder::ACCELERATION_BIAS);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(vins_mah::StateOrder::VELOCITY, vins_mah::StateOrder::GYROSCOPE_BIAS);

            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8) {
              VLOG(1) << "Numerical unstable in preintegration";
              //std::cout << pre_integration->jacobian << std::endl;
            }

            if (jacobians[0])
            {
              Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
              jacobian_pose_i.setZero();

              jacobian_pose_i.block<3, 3>(vins_mah::StateOrder::POSITION, vins_mah::StateOrder::POSITION) = -Qi.inverse().toRotationMatrix();
              jacobian_pose_i.block<3, 3>(vins_mah::StateOrder::POSITION, vins_mah::StateOrder::ROTATION) = vins_mah::utils::Rotations::skewSymmetric(Qi.inverse() * (0.5 * gravity_vector_ * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
              jacobian_pose_i.block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::ROTATION) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
              Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * vins_mah::utils::Rotations::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
              jacobian_pose_i.block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::ROTATION) = -(vins_mah::utils::Rotations::Qleft(Qj.inverse() * Qi) * vins_mah::utils::Rotations::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
#endif

              jacobian_pose_i.block<3, 3>(vins_mah::StateOrder::VELOCITY, vins_mah::StateOrder::ROTATION) = vins_mah::utils::Rotations::skewSymmetric(Qi.inverse() * (gravity_vector_ * sum_dt + Vj - Vi));

              jacobian_pose_i = sqrt_info * jacobian_pose_i;

              if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
              {
                VLOG(1) << "Numerical unstable in preintegration";
                //std::cout << sqrt_info << std::endl;
                //ROS_BREAK();
              }
            }
            if (jacobians[1])
            {
              Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
              jacobian_speedbias_i.setZero();
              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::POSITION, vins_mah::StateOrder::VELOCITY - vins_mah::StateOrder::VELOCITY) = -Qi.inverse().toRotationMatrix() * sum_dt;
              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::POSITION, vins_mah::StateOrder::ACCELERATION_BIAS - vins_mah::StateOrder::VELOCITY) = -dp_dba;
              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::POSITION, vins_mah::StateOrder::GYROSCOPE_BIAS - vins_mah::StateOrder::VELOCITY) = -dp_dbg;

#if 0
              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::GYROSCOPE_BIAS - vins_mah::StateOrder::VELOCITY) = -dq_dbg;
#else
              //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * vins_mah::utils::Rotations::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
              //jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::GYROSCOPE_BIAS - vins_mah::StateOrder::VELOCITY) = -vins_mah::utils::Rotations::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::GYROSCOPE_BIAS - vins_mah::StateOrder::VELOCITY) = -vins_mah::utils::Rotations::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
#endif

              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::VELOCITY, vins_mah::StateOrder::VELOCITY - vins_mah::StateOrder::VELOCITY) = -Qi.inverse().toRotationMatrix();
              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::VELOCITY, vins_mah::StateOrder::ACCELERATION_BIAS - vins_mah::StateOrder::VELOCITY) = -dv_dba;
              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::VELOCITY, vins_mah::StateOrder::GYROSCOPE_BIAS - vins_mah::StateOrder::VELOCITY) = -dv_dbg;

              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::ACCELERATION_BIAS, vins_mah::StateOrder::ACCELERATION_BIAS - vins_mah::StateOrder::VELOCITY) = -Eigen::Matrix3d::Identity();

              jacobian_speedbias_i.block<3, 3>(vins_mah::StateOrder::GYROSCOPE_BIAS, vins_mah::StateOrder::GYROSCOPE_BIAS - vins_mah::StateOrder::VELOCITY) = -Eigen::Matrix3d::Identity();

              jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

              //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
              //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
              Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
              jacobian_pose_j.setZero();

              jacobian_pose_j.block<3, 3>(vins_mah::StateOrder::POSITION, vins_mah::StateOrder::POSITION) = Qi.inverse().toRotationMatrix();

#if 0
              jacobian_pose_j.block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::ROTATION) = Eigen::Matrix3d::Identity();
#else
              Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * vins_mah::utils::Rotations::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
              jacobian_pose_j.block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::ROTATION) = vins_mah::utils::Rotations::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif

              jacobian_pose_j = sqrt_info * jacobian_pose_j;

              //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
              //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])
            {
              Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
              jacobian_speedbias_j.setZero();

              jacobian_speedbias_j.block<3, 3>(vins_mah::StateOrder::VELOCITY, vins_mah::StateOrder::VELOCITY - vins_mah::StateOrder::VELOCITY) = Qi.inverse().toRotationMatrix();

              jacobian_speedbias_j.block<3, 3>(vins_mah::StateOrder::ACCELERATION_BIAS, vins_mah::StateOrder::ACCELERATION_BIAS - vins_mah::StateOrder::VELOCITY) = Eigen::Matrix3d::Identity();

              jacobian_speedbias_j.block<3, 3>(vins_mah::StateOrder::GYROSCOPE_BIAS, vins_mah::StateOrder::GYROSCOPE_BIAS - vins_mah::StateOrder::VELOCITY) = Eigen::Matrix3d::Identity();

              jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

              // ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
              // ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
          }

          return true;
        }

        //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

        vins_mah::factor::IntegrationBase* pre_integration;
        Eigen::Vector3d gravity_vector_;

    };

  } // namespace factor

} // namespace vins_mah

#endif // VINS_MAH_FACTOR_IMU_H_
