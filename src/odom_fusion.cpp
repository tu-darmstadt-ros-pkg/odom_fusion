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
#include "odom_fusion/odom_fusion.h"
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (r,p,y,x,y,z)

OdomFusion::OdomFusion(const OdomFusionConfig& config, const Eigen::Vector3d& prior_position,
                       const Eigen::Quaterniond& prior_rotation, const Eigen::Vector3d& prior_velocity,
                       const ros::Time& initialization_time)
  : config_(config)
  , state_({ gtsam::NavState(gtsam::Pose3(gtsam::Rot3::Quaternion(prior_rotation.w(), prior_rotation.x(),
                                                                  prior_rotation.y(), prior_rotation.z()),
                                          prior_position),
                             prior_velocity),
             Eigen::Vector3d::Zero(), initialization_time })
  , zerovelocity_update_mode_(true)
  , rotation_correction_(config.initial_rotation_correction)
  , steady_state_support_(config.steady_state_minimum_duration)
  , initialized_(true)
{
  InitializeConfigParameters();
}

OdomFusion::OdomFusion(const OdomFusionConfig& config)
  : config_(config)
  , state_({ gtsam::NavState(gtsam::Pose3::identity(), gtsam::Vector3::Zero()), Eigen::Vector3d::Zero(), ros::Time(0) })
  , zerovelocity_update_mode_(true)
  , rotation_correction_(config.initial_rotation_correction)
  , steady_state_support_(config.steady_state_minimum_duration)
  , initialized_(false)
{
  InitializeConfigParameters();
}

void OdomFusion::InitializeConfigParameters()
{
  preintegration_params_ = gtsam::PreintegratedImuMeasurements::Params::MakeSharedU(config_.gravity_constant);
  preintegration_params_->accelerometerCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(config_.accel_noise_sigma, 2);  // acc white noise in continuous
  preintegration_params_->integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) * 1e-8;  // integration uncertainty continuous
  preintegration_params_->gyroscopeCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(config_.gyro_noise_sigma, 2);  // gyro white noise in continuous
  preintegration_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(preintegration_params_, bias_);
  bias_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(6, config_.prior_bias_sigma);
  velocity_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(3, config_.prior_velocity_sigma);
  pose_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << config_.prior_rotation_sigma, config_.prior_rotation_sigma, config_.prior_rotation_sigma,
       config_.prior_position_sigma, config_.prior_position_sigma, config_.prior_position_sigma)
          .finished());
}

void OdomFusion::AddIMUData(const ImuData& data)
{
  if (initialized_)
  {
    ImuData corrected_data = data;
    corrected_data.linear_acceleration = rotation_correction_ * data.linear_acceleration;
    corrected_data.angular_velocity = rotation_correction_ * data.angular_velocity;
    imu_queue_.push_back(corrected_data);
  }
  else
  {
    bool valid_orientation_from_msg = std::abs(data.orientation.norm() - 1.0) < 1E-5;
    Eigen::Quaterniond orientation =
        valid_orientation_from_msg ?
            data.orientation :
            Eigen::Quaterniond::FromTwoVectors(data.linear_acceleration.normalized(), Eigen::Vector3d::UnitZ());
    state_.nav_state_ =
        gtsam::NavState(gtsam::Pose3(gtsam::Rot3(orientation), gtsam::Point3::Zero()), gtsam::Vector3::Zero());
    state_.time_state_ = data.time;
    initialized_ = true;
    ROS_INFO("Initialization finished.");
  }
}

void OdomFusion::AddOdomData(const OdometryData& data)
{
  odom_queue_.push_back(data);
}

bool OdomFusion::Optimize(const ros::Time& query_time)
{
  if (imu_queue_.empty())
  {
    ROS_WARN("No IMU data, skipping optimization.");
    return false;
  }
  if (query_time <= imu_queue_.front().time)
  {
    ROS_WARN("Query time is earlier than first IMU data, skipping optimization.");
    return false;
  }
  preintegration_->resetIntegrationAndSetBias(bias_);
  graph_.resize(0);

  // Priors
  graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), state_.nav_state_.pose(), pose_noise_model_);
  graph_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), state_.nav_state_.v(), velocity_noise_model_);
  graph_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), bias_, bias_noise_model_);

  // IMU Preintegration factors
  bool finished_imu_integration = false;
  ros::Time imu_integration_time = state_.time_state_;

  bool imu_steady = true;
  bool odom_steady = true;

  while ((imu_queue_.size() > 1) && (imu_queue_[1].time <= state_.time_state_))
  {
    imu_queue_.pop_front();
  }
  Eigen::Vector3d angular_velocity_biased = imu_queue_.front().angular_velocity; // Low-pass filtered estimate of biased angular velocity in tracking frame 
  while (!finished_imu_integration)
  {
    finished_imu_integration = (imu_queue_.size() == 1) || (std::next(imu_queue_.begin())->time >= query_time);
    double integration_interval;

    if (imu_queue_.size() == 1 || std::next(imu_queue_.begin())->time >= query_time)
    {
      integration_interval = ros::Duration(query_time - imu_queue_.front().time).toSec();
      finished_imu_integration = true;
      imu_integration_time = query_time;
      angular_velocity_biased = 0.5 * angular_velocity_biased + 0.5 * imu_queue_.front().angular_velocity;
    }
    else
    {
      integration_interval = ros::Duration(std::next(imu_queue_.begin())->time - imu_integration_time).toSec();
      imu_integration_time = std::next(imu_queue_.begin())->time;
      angular_velocity_biased = 0.5 * angular_velocity_biased + 0.5 * imu_queue_.front().angular_velocity;
    }
    preintegration_->integrateMeasurement(imu_queue_.front().linear_acceleration, imu_queue_.front().angular_velocity,
                                          integration_interval);
    if (std::abs(imu_queue_.front().angular_velocity.x()) > config_.steady_state_imu_angular_velocity_threshold ||
        std::abs(imu_queue_.front().angular_velocity.y()) > config_.steady_state_imu_angular_velocity_threshold ||
        std::abs(imu_queue_.front().angular_velocity.z()) > config_.steady_state_imu_angular_velocity_threshold)
    {
      imu_steady = false;
    }
    if ((imu_queue_.size() > 1) && (imu_queue_[1].time <= query_time))
    {
      imu_queue_.pop_front();
    }
  }
  gtsam::ImuFactor imu_factor(X(0), V(0), X(1), V(1), B(0), *preintegration_);
  graph_.add(imu_factor);
  gtsam::imuBias::ConstantBias zero_bias(gtsam::Vector3(0, 0, 0), gtsam::Vector3(0, 0, 0));
  graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(0), B(1), zero_bias, bias_noise_model_));


  // IMU Orientation factor
  if (config_.use_imu_rollpitch)
  {
    gtsam::Rot3 imu_orientation =
        gtsam::Rot3::Quaternion(imu_queue_.front().orientation.w(), imu_queue_.front().orientation.x(),
                                imu_queue_.front().orientation.y(), imu_queue_.front().orientation.z());
    gtsam::Pose3 state_imu_ = gtsam::Pose3(imu_orientation, gtsam::Vector3::Zero());
    std::vector<size_t> roll_pitch_indices = { 0, 1 };
    gtsam::noiseModel::Diagonal::shared_ptr pitch_roll_noise_2d =
        gtsam::noiseModel::Isotropic::Sigma(2, config_.imu_rollpitch_sigma);
    gtsam::Vector2 roll_pitch = gtsam::Vector2(state_imu_.rotation().roll(), state_imu_.rotation().pitch());
    graph_.emplace_shared<gtsam::PartialPriorFactor<gtsam::Pose3>>(X(1), roll_pitch_indices, roll_pitch,
                                                                   pitch_roll_noise_2d);
  }

  // Odom factor
  while (odom_queue_.size() > 1 && std::next(odom_queue_.begin())->time <= query_time)
  {
    odom_queue_.pop_front();
  }

  {
    gtsam::Vector3 v_linear_local_measurement;
    gtsam::Vector3 v_angular_local_measurement;
    if (odom_queue_.size() == 1 || odom_queue_.begin()->time == query_time)
    {
      v_linear_local_measurement = odom_queue_.begin()->linear_velocity;
      v_angular_local_measurement = odom_queue_.begin()->angular_velocity;
    }
    else
    {
      double delta = ros::Duration(std::next(odom_queue_.begin())->time - odom_queue_.begin()->time).toSec();
      double weight_0 = ros::Duration(query_time - odom_queue_.begin()->time).toSec() / delta;
      double weight_1 = ros::Duration(std::next(odom_queue_.begin())->time - query_time).toSec() / delta;
      v_linear_local_measurement =
          weight_0 * odom_queue_.begin()->linear_velocity + weight_1 * std::next(odom_queue_.begin())->linear_velocity;
      v_angular_local_measurement = weight_0 * odom_queue_.begin()->angular_velocity +
                                    weight_1 * std::next(odom_queue_.begin())->angular_velocity;
    }

    if (std::abs(v_linear_local_measurement.x()) > config_.steady_state_odom_threshold ||
        std::abs(v_angular_local_measurement.z()) > config_.steady_state_odom_threshold)
    {
      odom_steady = false;
    }
    if (imu_steady && odom_steady)
    {
      steady_state_support_ += ros::Duration(query_time - state_.time_state_).toSec();
    }
    else
    {
      steady_state_support_ = 0.0;
    }
    zerovelocity_update_mode_ = steady_state_support_ > config_.steady_state_minimum_duration;

    if (!zerovelocity_update_mode_)
    {
      gtsam::Expression<gtsam::Pose3> x_next(X(1));
      gtsam::Expression<gtsam::Vector3> v_next(V(1));
      auto v_local = gtsam::Expression<gtsam::Vector3>(gtsam::unrotate(gtsam::rotation(x_next), v_next));
      double sigma_v_approximated =
          config_.velocity_sigma_scale * (v_linear_local_measurement.norm() + config_.velocity_sigma_normalization);
      gtsam::noiseModel::Diagonal::shared_ptr local_velocity_noise =
          gtsam::noiseModel::Isotropic::Sigma(3, sigma_v_approximated);
      graph_.addExpressionFactor(v_local, v_linear_local_measurement, local_velocity_noise);
    }
    else
    {
      Eigen::Matrix<double, 6, 1> pose_sigmas;
      pose_sigmas << config_.steady_rxy_sigma, config_.steady_rxy_sigma, config_.steady_rz_pxyz_sigma, config_.steady_rz_pxyz_sigma, config_.steady_rz_pxyz_sigma, config_.steady_rz_pxyz_sigma;
      gtsam::noiseModel::Diagonal::shared_ptr zero_velocity_pose_noise_model_ =
          gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas);

      gtsam::noiseModel::Isotropic::shared_ptr zero_velocity_velocity_noise_model_ =
          gtsam::noiseModel::Isotropic::Sigma(3, 1.0e-6);
      gtsam::Pose3 zero_pose = gtsam::Pose3::identity();
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(0), X(1), zero_pose, zero_velocity_pose_noise_model_));
      graph_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(1), gtsam::Vector3::Zero(),
                                                                zero_velocity_velocity_noise_model_);
    }
  }

  // Optimize
  gtsam::Values initial_values;
  gtsam::NavState propagated_state = preintegration_->predict(state_.nav_state_, bias_);
  initial_values.insert(X(0), state_.nav_state_.pose());
  initial_values.insert(V(0), state_.nav_state_.v());
  initial_values.insert(B(0), bias_);
  initial_values.insert(B(1), bias_);
  if (zerovelocity_update_mode_)
  {
    initial_values.insert(X(1), state_.nav_state_.pose());
    initial_values.insert(V(1), state_.nav_state_.v());
  }
  else
  {
    initial_values.insert(X(1), propagated_state.pose());
    initial_values.insert(V(1), propagated_state.v());
  }


  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_values);
  result_ = optimizer.optimize();


  if (config_.propagate_variance)
  {
    gtsam::Marginals marginals(graph_, result_);
    pose_noise_model_ = gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(X(1))); //crashes here
    velocity_noise_model_ = gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(V(1)));
    bias_noise_model_ = gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(B(1)));
  }

  // Update State
  state_.nav_state_ = gtsam::NavState(result_.at<gtsam::Pose3>(X(1)), result_.at<gtsam::Vector3>(V(1)));
  bias_ = result_.at<gtsam::imuBias::ConstantBias>(B(1));
  state_.angular_velocity_tracking_frame_ = angular_velocity_biased - bias_.gyroscope();
  state_.time_state_ = query_time;
  return true;
}

// Static IMU Calibration, assumes the IMU to be static during calibration
// interval. Estimates acceleration scale, angular velocity biases and aligns
// the IMU frame with the direction of gravity.
void OdomFusion::CalibrateOrientation(const std::deque<ImuData>& imu_data, const double gravity_constant,
                                      Eigen::Transform<double, 3, Eigen::Affine>& orientation_correction)
{
  Eigen::Vector3d angular_biases = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerations = Eigen::Vector3d::Zero();
  for (const auto& imu_observation : imu_data)
  {
    accelerations += imu_observation.linear_acceleration;
  }
  double normalization_factor = 1.0 / double(imu_data.size());
  accelerations = normalization_factor * accelerations;
  std::cout << "IMU data before calibration:\n  Linear accelerations:\n"
            << accelerations << "\n Acceleration scale: \n"
            << accelerations.norm() / gravity_constant << "\n Data duration: \n"
            << ros::Duration(imu_data.back().time - imu_data.front().time).toSec() << "\n Data set size: \n"
            << imu_data.size() << std::endl;
  double acceleration_scale_correction_factor = gravity_constant / accelerations.norm();
  Eigen::Quaterniond r =
      Eigen::Quaterniond::FromTwoVectors(accelerations.normalized(), Eigen::Vector3d({ 0.0, 0.0, 1.0 }));

  orientation_correction = acceleration_scale_correction_factor * r.matrix();

  std::cout << "Orientation correction:\n"
            << orientation_correction.rotation() << "\n"
            << orientation_correction.translation() << std::endl;
  std::cout << "IMU data after calibration:\n  Linear accelerations:\n"
            << orientation_correction * accelerations << "\n Acceleration scale: \n"
            << (orientation_correction * accelerations).norm() / gravity_constant << std::endl;
}

gtsam::Matrix OdomFusion::GetPoseCovariance() const
{
  return gtsam::Marginals(graph_, result_).marginalCovariance(X(1));
}

gtsam::Matrix OdomFusion::GetVelocityCovariance() const
{
  return gtsam::Marginals(graph_, result_).marginalCovariance(V(1));
}

gtsam::Matrix OdomFusion::GetBiasCovariance() const
{
  return gtsam::Marginals(graph_, result_).marginalCovariance(B(1));
}

const State& OdomFusion::GetState() const
{
  return state_;
}
const ros::Time& OdomFusion::GetTimeState() const
{
  return state_.time_state_;
}
const gtsam::imuBias::ConstantBias& OdomFusion::GetBias() const
{
  return bias_;
}
State& OdomFusion::GetMutableState()
{
  return state_;
}
ros::Time& OdomFusion::GetMutableTimeState()
{
  return state_.time_state_;
}
bool OdomFusion::IsInitialized()
{
  return initialized_;
}

bool OdomFusion::ZeroVelocityUpdateMode() const
{
  return zerovelocity_update_mode_;
};
void OdomFusion::SetZeroVelocityUpdateMode(bool mode)
{
  zerovelocity_update_mode_ = mode;
  if (!mode && !imu_calibration_data_.empty())
  {
    Eigen::Transform<double, 3, Eigen::Affine> orientation_calibration;
    //    CalibrateOrientation(imu_calibration_data_, config_.gravity_constant, orientation_calibration);
  }
};


ros::Time OdomFusion::FullSensorDataUntil() {
  if(imu_queue_.empty() || odom_queue_.empty()) {
    return ros::Time(0);
  }

  ros::Time latest_imu = imu_queue_.back().time;
  ros::Time latest_odom = odom_queue_.back().time;
  if(latest_imu > latest_odom) {
    return latest_odom;
  } else {
    return latest_imu;
  }
}
