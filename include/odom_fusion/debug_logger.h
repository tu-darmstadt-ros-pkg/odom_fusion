//=================================================================================================
// Copyright (c) 2021, Kevin Daun, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef ODOM_FUSION_DEBUG_LOGGER_H_
#define ODOM_FUSION_DEBUG_LOGGER_H_
#include <iostream>
#include <fstream>
#include <string>

#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/expressions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "ros_utils.h"
class DebugLogger
{
public:
  double accumulated_error;
  size_t num_entries;
  explicit DebugLogger(const std::string& filename) : accumulated_error(0.0), num_entries(0)
  {
    std::cout << "Logging to " << filename << std::endl;
    file_.open(filename);
    file_ << "t,estimate_p_x,estimate_p_y,estimate_p_z,estimate_v_x,estimate_v_y,estimate_v_z,estimate_roll,estimate_"
             "pitch,estimate_yaw,odom_p_x,odom_p_y,odom_p_z,odom_v_x,odom_v_y,odom_v_z,ref_p_x,ref_p_y,ref_p_z,ref_v_x,"
             "ref_v_y,ref_v_z,ref_roll,ref_pitch,ref_yaw,c_rx,c_ry,c_rz,c_px,c_py,c_pz,c_vx,c_vy,c_vz,c_bax,c_bay,c_baz,c_bgx,c_bgy,c_bgz,bias_"
             "acc_x,bias_acc_y,bias_"
             "acc_z,bias_gyro_x,bias_gyro_y,bias_gyro_z,steady_state, error\n";
  }

  void AddEntry(double time, const gtsam::NavState& estimate, const nav_msgs::Odometry& measurement,
                const nav_msgs::Odometry& reference, const gtsam::Matrix& cov_x, const gtsam::Matrix& cov_v,
                const gtsam::Matrix& cov_b, const gtsam::imuBias::ConstantBias& imu_bias, bool steady_state)
  {
    num_entries++;
    double error = std::sqrt(std::pow(reference.pose.pose.position.x - estimate.t().x(), 2) +
                             std::pow(reference.pose.pose.position.y - estimate.t().y(), 2) +
                             std::pow(reference.pose.pose.position.z - estimate.t().z(), 2));
    accumulated_error += error;
    gtsam::Matrix cx = cov_x.diagonal().transpose();
    gtsam::Matrix cv = cov_v.diagonal().transpose();
    gtsam::Matrix cb = cov_b.diagonal().transpose();


    gtsam::Rot3 ref_orientation =
        gtsam::Rot3::Quaternion(reference.pose.pose.orientation.w, reference.pose.pose.orientation.x,
        reference.pose.pose.orientation.y, reference.pose.pose.orientation.z);

    file_ << time << "," << estimate.t().x() << "," << estimate.t().y() << "," << estimate.t().z() << ","
          << estimate.bodyVelocity().x() << "," << estimate.bodyVelocity().y() << "," << estimate.bodyVelocity().z()
          << "," << gtsam::Rot3(estimate.R()).roll() << "," << gtsam::Rot3(estimate.R()).pitch() << ","
          << gtsam::Rot3(estimate.R()).yaw() << "," << measurement.pose.pose.position.x << ","
          << measurement.pose.pose.position.y << "," << measurement.pose.pose.position.z << ","
          << measurement.twist.twist.linear.x << "," << measurement.twist.twist.linear.y << ","
          << measurement.twist.twist.linear.z << "," << reference.pose.pose.position.x << ","
          << reference.pose.pose.position.y << "," << reference.pose.pose.position.z << ","
          << reference.twist.twist.linear.x << "," << reference.twist.twist.linear.y << ","
          << reference.twist.twist.linear.z << "," << ref_orientation.roll() << "," << ref_orientation.pitch() << ","
        << ref_orientation.yaw() << "," << cx(0) << "," << cx(1) << "," << cx(2) << "," << cx(3) << ","
          << cx(4) << "," << cx(5) << "," << cv(0) << "," << cv(1) << "," << cv(2) << "," << cb(0) << "," << cb(1)
          << "," << cb(2) << "," << cb(3) << "," << cb(4) << "," << cb(5) << "," << imu_bias.accelerometer().x() << ","
          << imu_bias.accelerometer().y() << "," << imu_bias.accelerometer().z() << "," << imu_bias.gyroscope().x()
          << "," << imu_bias.gyroscope().y() << "," << imu_bias.gyroscope().z() << "," << int(steady_state) << ","
          << error << "\n";
  }

  ~DebugLogger()
  {
    if (num_entries > 0)
      ROS_INFO("error %f", accumulated_error / double(num_entries));
    file_.close();
  }

  std::ofstream file_;
};
#endif  // ODOM_FUSION_DEBUG_LOGGER_H_