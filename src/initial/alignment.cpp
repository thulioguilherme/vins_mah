/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "vins_mah/initial/alignment.h"

namespace vins_mah
{
  namespace initial
  {

    /* solveGyroscopeBias() //{ */

    void solveGyroscopeBias(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, int window_size) {
      Eigen::Matrix3d A;
      Eigen::Vector3d b;
      Eigen::Vector3d delta_bg;
      A.setZero();
      b.setZero();
      std::map<double, ImageFrame>::iterator frame_i;
      std::map<double, ImageFrame>::iterator frame_j;
      for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++) {
        frame_j = next(frame_i);
        Eigen::MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        Eigen::VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(vins_mah::StateOrder::ROTATION, vins_mah::StateOrder::GYROSCOPE_BIAS);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
      }
      delta_bg = A.ldlt().solve(b);
      /* LOG(INFO) << "Gyroscope bias initial calibration: " << delta_bg.transpose(); */
      std::cout << "Gyroscope bias initial calibration: " << delta_bg.transpose() << std::endl;

      for (int i = 0; i <= window_size; i++) {
        Bgs[i] += delta_bg;
      }

      for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++) {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
      }
    }

    //}

    /* tangentBasis() //{ */

    Eigen::MatrixXd tangentBasis(Eigen::Vector3d &g0) {
      Eigen::Vector3d b, c;
      Eigen::Vector3d a = g0.normalized();
      Eigen::Vector3d tmp(0, 0, 1);
      if (a == tmp) {
        tmp << 1, 0, 0;
      }

      b = (tmp - a * (a.transpose() * tmp)).normalized();
      c = a.cross(b);
      Eigen::MatrixXd bc(3, 2);
      bc.block<3, 1>(0, 0) = b;
      bc.block<3, 1>(0, 1) = c;

      return bc;
    }

    //}

    /* refineGravity() //{ */

    void refineGravity(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &estimated_gravity, Eigen::VectorXd &x, Eigen::Vector3d gravity_reference, Eigen::Vector3d T_ItoC) {
      Eigen::Vector3d g0 = estimated_gravity.normalized() * gravity_reference.norm();
      Eigen::Vector3d lx, ly;

      //VectorXd x;
      int all_frame_count = all_image_frame.size();
      int n_state = all_frame_count * 3 + 2 + 1;

      Eigen::MatrixXd A{n_state, n_state};
      A.setZero();
      Eigen::VectorXd b{n_state};
      b.setZero();

      std::map<double, ImageFrame>::iterator frame_i;
      std::map<double, ImageFrame>::iterator frame_j;
      for (int k = 0; k < 4; k++) {
        MatrixXd lxly(3, 2);
        lxly = tangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++) {
          frame_j = next(frame_i);

          Eigen::MatrixXd tmp_A(6, 9);
          tmp_A.setZero();
          Eigen::VectorXd tmp_b(6);
          tmp_b.setZero();

          double dt = frame_j->second.pre_integration->sum_dt;

          tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
          tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
          tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
          tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * T_ItoC - T_ItoC - frame_i->second.R.transpose() * dt * dt / 2 * g0;
          tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
          tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
          tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
          tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;

          Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
          //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
          //MatrixXd cov_inv = cov.inverse();
          cov_inv.setIdentity();

          Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
          Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

          A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
          b.segment<6>(i * 3) += r_b.head<6>();

          A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
          b.tail<3>() += r_b.tail<3>();

          A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
          A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        A = A * 1000.0;

        b = b * 1000.0;
        x = A.ldlt().solve(b);
        VectorXd dg = x.segment<2>(n_state - 3);
        g0 = (g0 + lxly * dg).normalized() * gravity_reference.norm();
        //double s = x(n_state - 1);
      }

      estimated_gravity = g0;
    }

    //}

    /* linearAlignment() //{ */

    bool linearAlignment(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &estimated_gravity, Eigen::VectorXd &x, Eigen::Vector3d gravity_reference, Eigen::Vector3d T_ItoC) {
      int all_frame_count = all_image_frame.size();
      int n_state = all_frame_count * 3 + 3 + 1;

      Eigen::MatrixXd A{n_state, n_state};
      A.setZero();
      Eigen::VectorXd b{n_state};
      b.setZero();

      std::map<double, ImageFrame>::iterator frame_i;
      std::map<double, ImageFrame>::iterator frame_j;
      int i = 0;
      for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++) {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;

        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * T_ItoC - T_ItoC;
        //cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();

        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
      }

      A = A * 1000.0;
      b = b * 1000.0;
      x = A.ldlt().solve(b);
      double s = x(n_state - 1) / 100.0;
      VLOG(1) << "Estimated scale: " << s;
      estimated_gravity = x.segment<3>(n_state - 4);
      VLOG(1) << "Result g " << estimated_gravity.norm() << " " << estimated_gravity.transpose();

      if (fabs(estimated_gravity.norm() - gravity_reference.norm()) > 0.5 || s < 0) {
        return false;
      }

      refineGravity(all_image_frame, estimated_gravity, x, gravity_reference, T_ItoC);

      s = (x.tail<1>())(0) / 100.0;
      (x.tail<1>())(0) = s;
      VLOG(1) << "Refined gravity: " << estimated_gravity.norm() << " " << estimated_gravity.transpose();

      if (s < 0.0) {
        return false;
      } else {
        return true;
      }
    }

    //}

    /* visualImuAlignment() //{ */

    bool visualImuAlignment(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &estimated_gravity, Eigen::VectorXd &x, int window_size, Eigen::Vector3d gravity_reference, Eigen::Vector3d T_ItoC) {
      solveGyroscopeBias(all_image_frame, Bgs, window_size);

      if (linearAlignment(all_image_frame, estimated_gravity, x, gravity_reference, T_ItoC)) {
        return true;
      } else {
        return false;
      }
    }

    //}

  } // namespace initial

} // namespace vins_mah
