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

#ifndef ODOM_FUSION_ODOM_FUSION_H_
#define ODOM_FUSION_ODOM_FUSION_H_
#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <queue>

struct OdomFusionConfig
{
public:
  // Default params mostly from https://github.com/haidai/gtsam/blob/master/gtsam/navigation/CombinedImuFactor.h
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double gravity_constant = 9.80511;
  double prior_bias_sigma = 1.0e-3;
  double prior_velocity_sigma = 1.0e-3;
  double prior_position_sigma = 1.0e-3;
  double prior_rotation_sigma = 1.0e-2;
  double velocity_sigma_scale = 1.0e-1;
  double velocity_sigma_normalization = 1.0e-4;
  bool propagate_variance = false;
  bool use_imu_rollpitch = false;
  double imu_rollpitch_sigma = 3.0e-1;
  Eigen::Matrix3d initial_rotation_correction = Eigen::Matrix3d::Identity();
  double steady_state_odom_threshold = 1.0e-5;
  double steady_state_imu_angular_velocity_threshold = 1.5e-2;
  double steady_state_minimum_duration = 0.25;
  double steady_rxy_sigma = 1.0e-5;
  double steady_rz_pxyz_sigma = 1.0e-6;
  double integration_dt = 0.02;
  std::string tracking_frame_ = "imu_link";
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";
};

struct OdomFusionOfflineConfig
{
public:
  std::string logging_filename = "/motion_capture/odom";
  int log_downsampling = 1;
  int print_downsampling = 1000;
  std::string offline_imu_topic = "/imu/data";
  std::string offline_odom_raw_topic = "/odom_raw";
  std::string offline_reference_topic = "/motion_capture/odom";
  bool use_reference_orientation = true;
};

struct State
{
  gtsam::NavState nav_state_;
  Eigen::Vector3d angular_velocity_tracking_frame_;
  ros::Time time_state_;
};

struct ImuData
{
  ros::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
  Eigen::Quaterniond orientation;
};

struct OdometryData
{
  ros::Time time;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;
};

class OdomFusion
{
public:
  OdomFusion(const OdomFusionConfig& config);

  OdomFusion(const OdomFusionConfig& config, const Eigen::Vector3d& prior_position,
             const Eigen::Quaterniond& prior_rotation, const Eigen::Vector3d& prior_velocity,
             const ros::Time& initialization_time);

  void AddIMUData(const ImuData& data);

  void AddOdomData(const OdometryData& data);

  bool Optimize(const ros::Time& time);

  void CalibrateOrientation(const std::deque<ImuData>& imu_data, double gravity_constant,
                            Eigen::Transform<double, 3, Eigen::Affine>& orientation_correction);

  gtsam::Matrix GetPoseCovariance() const;
  gtsam::Matrix GetVelocityCovariance() const;
  gtsam::Matrix GetBiasCovariance() const;

  const State& GetState() const;
  const ros::Time& GetTimeState() const;
  State& GetMutableState();
  ros::Time& GetMutableTimeState();
  bool IsInitialized();
  const gtsam::imuBias::ConstantBias& GetBias() const;
  bool ZeroVelocityUpdateMode() const;
  void SetZeroVelocityUpdateMode(bool mode);
  ros::Time FullSensorDataUntil();

private:
  void InitializeConfigParameters();
  OdomFusionConfig config_;

  gtsam::noiseModel::Gaussian::shared_ptr pose_noise_model_;
  gtsam::noiseModel::Gaussian::shared_ptr velocity_noise_model_;
  gtsam::noiseModel::Gaussian::shared_ptr bias_noise_model_;

  boost::shared_ptr<gtsam::PreintegratedImuMeasurements::Params> preintegration_params_;
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> preintegration_;

  std::deque<ImuData> imu_queue_;
  std::deque<OdometryData> odom_queue_;
  std::deque<ImuData> imu_calibration_data_;

  State state_;
  gtsam::imuBias::ConstantBias bias_;
  gtsam::ExpressionFactorGraph graph_;
  gtsam::Values result_;

  Eigen::Matrix3d rotation_correction_;

  bool zerovelocity_update_mode_;
  double steady_state_support_;
  bool initialized_;
};

#endif  // ODOM_FUSION_ODOM_FUSION_H_
