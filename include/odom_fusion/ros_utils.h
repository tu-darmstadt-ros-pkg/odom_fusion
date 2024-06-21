
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

#ifndef ODOM_FUSION_ROS_UTILS_H_
#define ODOM_FUSION_ROS_UTILS_H_

#include <odom_fusion/odom_fusion.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

OdometryData FromROSMsg(const nav_msgs::Odometry& odom_data)
{
  Eigen::Vector3d position(odom_data.pose.pose.position.x, odom_data.pose.pose.position.y,
                           odom_data.pose.pose.position.z);
  Eigen::Quaterniond orientation(odom_data.pose.pose.orientation.w, odom_data.pose.pose.orientation.x,
                                 odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z);
  Eigen::Vector3d linear_velocity(odom_data.twist.twist.linear.x, odom_data.twist.twist.linear.y,
                                  odom_data.twist.twist.linear.z);
  Eigen::Vector3d angular_velocity(odom_data.twist.twist.angular.x, odom_data.twist.twist.angular.y,
                                   odom_data.twist.twist.angular.z);
  return OdometryData({ odom_data.header.stamp, position, orientation, linear_velocity, angular_velocity });
}

ImuData FromROSMsg(const sensor_msgs::Imu& imu_data)
{
  Eigen::Vector3d linear_acceleration(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y,
                                      imu_data.linear_acceleration.z);
  Eigen::Vector3d angular_velocity(imu_data.angular_velocity.x, imu_data.angular_velocity.y,
                                   imu_data.angular_velocity.z);
  Eigen::Quaterniond orientation(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y,
                                 imu_data.orientation.z);
  return ImuData({ imu_data.header.stamp, linear_acceleration, angular_velocity, orientation });
}

geometry_msgs::Transform ToTransform(const gtsam::NavState& state)
{
  geometry_msgs::Transform transform;
  transform.translation.x = state.t().x();
  transform.translation.y = state.t().y();
  transform.translation.z = state.t().z();
  transform.rotation.w = state.quaternion().w();
  transform.rotation.x = state.quaternion().x();
  transform.rotation.y = state.quaternion().y();
  transform.rotation.z = state.quaternion().z();
  return transform;
}

geometry_msgs::Transform ToTransform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation)
{
  geometry_msgs::Transform transform;
  transform.translation.x = translation.x();
  transform.translation.y = translation.y();
  transform.translation.z = translation.z();
  transform.rotation.w = rotation.w();
  transform.rotation.x = rotation.x();
  transform.rotation.y = rotation.y();
  transform.rotation.z = rotation.z();
  return transform;
}

geometry_msgs::Pose ToPose(const gtsam::NavState& state)
{
  geometry_msgs::Pose pose;
  pose.position.x = state.t().x();
  pose.position.y = state.t().y();
  pose.position.z = state.t().z();
  pose.orientation.w = state.quaternion().w();
  pose.orientation.x = state.quaternion().x();
  pose.orientation.y = state.quaternion().y();
  pose.orientation.z = state.quaternion().z();
  return pose;
}

geometry_msgs::Pose ToPose(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation)
{
  geometry_msgs::Pose pose;
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  pose.orientation.w = rotation.w();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  return pose;
}

geometry_msgs::Twist ToTwist(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear.x();
  twist.linear.y = linear.y();
  twist.linear.z = linear.z();
  twist.angular.x = angular.x();
  twist.angular.y = angular.y();
  twist.angular.z = angular.z();
  return twist;
}

template <typename T>
Eigen::Vector3d ToVector3d(const T& in)
{
  return { in.x, in.y, in.z };
}

template <typename T>
Eigen::Quaterniond ToQuaterniond(const T& in)
{
  return { in.w, in.x, in.y, in.z };
}

gtsam::NavState ToNavState(const nav_msgs::Odometry& msg)
{
  const Eigen::Vector3d& state_t = ToVector3d(msg.pose.pose.position);
  Eigen::Quaterniond state_R = ToQuaterniond(msg.pose.pose.orientation);
  // Odometry message expects linear velocity in tracking frame wheras gtsam expects the global frame
  Eigen::Vector3d state_v = state_R * ToVector3d(msg.twist.twist.linear);
  return { gtsam::Rot3::Quaternion(state_R.w(), state_R.x(), state_R.y(), state_R.z()), state_t, state_v };
}

void LoadBag(const std::string& bag_filename, const OdomFusionOfflineConfig& config,
             std::vector<sensor_msgs::Imu>& imu_data, std::vector<nav_msgs::Odometry>& odom_data,
             std::vector<nav_msgs::Odometry>& fused_odom_data)
{
  std::cout << "Reading bag " << bag_filename << std::endl;
  rosbag::Bag bag;
  bag.open(bag_filename);
  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    if (m.getTopic() == config.offline_imu_topic)
    {
      sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
      if (imu != nullptr)
      {
        imu_data.push_back(*imu);
      }
    }
    if (m.getTopic() == config.offline_odom_raw_topic)
    {
      nav_msgs::OdometryPtr wheel = m.instantiate<nav_msgs::Odometry>();
      if (wheel != nullptr)
      {
        odom_data.push_back(*wheel);
      }
    }
    if (m.getTopic() == config.offline_reference_topic)
    {
      nav_msgs::OdometryPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
      if (odom_msg != nullptr)
      {
        fused_odom_data.push_back(*odom_msg);
      }
    }
  }
  std::cout << "Read " << imu_data.size() << " imu messages from topic " << config.offline_imu_topic << std::endl;
  std::cout << "Read " << odom_data.size() << " odom messages from topic " << config.offline_odom_raw_topic
            << std::endl;
  std::cout << "Read " << fused_odom_data.size() << " fused_odom messages from topic " << config.offline_reference_topic
            << std::endl;
  bag.close();
}

void LoadParams(OdomFusionOfflineConfig& config, ros::NodeHandle& nh)
{
  nh.param<std::string>("odom_fusion/logging_filename", config.logging_filename, config.logging_filename);
  nh.param<int>("odom_fusion/log_downsampling", config.log_downsampling, config.log_downsampling);
  nh.param<int>("odom_fusion/print_downsampling", config.print_downsampling, config.print_downsampling);
  nh.param<std::string>("odom_fusion/offline_imu_topic", config.offline_imu_topic, config.offline_imu_topic);
  nh.param<std::string>("odom_fusion/offline_odom_raw_topic", config.offline_odom_raw_topic,
                        config.offline_odom_raw_topic);
  nh.param<std::string>("odom_fusion/offline_reference_topic", config.offline_reference_topic,
                        config.offline_reference_topic);
  nh.param<bool>("odom_fusion/use_reference_orientation", config.use_reference_orientation,
                 config.use_reference_orientation);
}

void LoadParams(OdomFusionConfig& config, ros::NodeHandle& nh)
{
  // Priors
  nh.param<double>("odom_fusion/prior_velocity_sigma", config.prior_velocity_sigma, config.prior_velocity_sigma);
  nh.param<double>("odom_fusion/prior_position_sigma", config.prior_position_sigma, config.prior_position_sigma);
  nh.param<double>("odom_fusion/prior_rotation_sigma", config.prior_rotation_sigma, config.prior_rotation_sigma);

  // Preintegration
  nh.param<double>("odom_fusion/accel_noise_sigma", config.accel_noise_sigma, config.accel_noise_sigma);
  nh.param<double>("odom_fusion/gyro_noise_sigma", config.gyro_noise_sigma, config.gyro_noise_sigma);
  nh.param<double>("odom_fusion/prior_bias_sigma", config.prior_bias_sigma, config.prior_bias_sigma);
  nh.param<double>("odom_fusion/gravity_constant", config.gravity_constant, config.gravity_constant);
  nh.param<bool>("odom_fusion/use_imu_rollpitch", config.use_imu_rollpitch, config.use_imu_rollpitch);
  nh.param<double>("odom_fusion/imu_rollpitch_sigma", config.imu_rollpitch_sigma, config.imu_rollpitch_sigma);

  // Wheel/track odom
  nh.param<double>("odom_fusion/velocity_sigma_scale", config.velocity_sigma_scale, config.velocity_sigma_scale);
  nh.param<double>("odom_fusion/velocity_sigma_normalization", config.velocity_sigma_normalization);

  // General
  nh.param<bool>("odom_fusion/propagate_variance", config.propagate_variance, config.propagate_variance);
  nh.param<double>("odom_fusion/integration_dt", config.integration_dt, config.integration_dt);
  nh.param<std::string>("odom_fusion/base_link_frame", config.base_link_frame_, config.base_link_frame_);
  nh.param<std::string>("odom_fusion/odom_frame", config.odom_frame_, config.odom_frame_);
  nh.param<std::string>("odom_fusion/tracking_frame", config.tracking_frame_, config.tracking_frame_);

  nh.param<double>("odom_fusion/steady_state_odom_threshold", config.steady_state_odom_threshold,
                   config.steady_state_odom_threshold);
  nh.param<double>("odom_fusion/steady_state_imu_angular_velocity_threshold",
                   config.steady_state_imu_angular_velocity_threshold,
                   config.steady_state_imu_angular_velocity_threshold);
  nh.param<double>("odom_fusion/steady_state_minimum_duration", config.steady_state_minimum_duration,
                   config.steady_state_minimum_duration);
  nh.param<double>("odom_fusion/steady_rxy_sigma", config.steady_rxy_sigma,
                   config.steady_rxy_sigma);
  nh.param<double>("odom_fusion/steady_rz_pxyz_sigma", config.steady_rz_pxyz_sigma,
                   config.steady_rz_pxyz_sigma);
}

template <typename nav_msgs_odometry_type>
bool TransformOdomDataToOdomFrame(const nav_msgs_odometry_type& msg, const tf2_ros::Buffer& tf_buffer,
                                  const OdomFusionConfig& config, OdometryData& odom_data_tracking_frame)
{
  const OdometryData odom_data_odom_frame = FromROSMsg(*msg);
  geometry_msgs::TransformStamped transform_tracking_odom;
  geometry_msgs::TransformStamped transform_base_tracking;
  try
  {
    transform_tracking_odom =
        tf_buffer.lookupTransform(config.tracking_frame_, msg->child_frame_id, msg->header.stamp, ros::Duration(1.0));
    transform_base_tracking = tf_buffer.lookupTransform(config.base_link_frame_, config.tracking_frame_,
                                                        msg->header.stamp, ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  odom_data_tracking_frame.time = odom_data_odom_frame.time;
  Eigen::Vector3d t = ToVector3d(transform_tracking_odom.transform.translation);
  Eigen::Quaterniond R = ToQuaterniond(transform_tracking_odom.transform.rotation);
  Eigen::Vector3d omega = odom_data_odom_frame.angular_velocity;  // TODO: Rather use filtered IMU estimate
  Eigen::Matrix<double, 3, 3> omega_skrew;
  omega_skrew << 0.0, -omega[2], omega[1], omega[2], 0.0, -omega[0], -omega[1], omega[0], 0.0;
  odom_data_tracking_frame.orientation = R * odom_data_odom_frame.orientation;
  odom_data_tracking_frame.position = t + R * odom_data_tracking_frame.position;
  Eigen::Vector3d t_base_tracking = ToVector3d(transform_base_tracking.transform.translation);
  odom_data_tracking_frame.linear_velocity = R * (odom_data_odom_frame.linear_velocity + omega_skrew * t_base_tracking);
  odom_data_tracking_frame.angular_velocity = R * odom_data_odom_frame.angular_velocity;
  return true;
}

bool TransformStateToBaseFrame(const State& state, const tf2_ros::Buffer& tf_buffer, const OdomFusionConfig& config,
                               const ros::Time& result_time, geometry_msgs::TransformStamped& transform_odom_base,
                               nav_msgs::Odometry& out_msg)
{
  geometry_msgs::TransformStamped transform_tracking_base;
  geometry_msgs::TransformStamped transform_base_tracking;
  try
  {
    transform_tracking_base =
        tf_buffer.lookupTransform(config.tracking_frame_, config.base_link_frame_, result_time, ros::Duration(1.0));
    transform_base_tracking =
        tf_buffer.lookupTransform(config.base_link_frame_, config.tracking_frame_, result_time, ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  geometry_msgs::TransformStamped transform_odom_tracking;
  transform_odom_tracking.header.stamp = result_time;
  transform_odom_tracking.header.frame_id = config.odom_frame_;
  transform_odom_tracking.child_frame_id = config.tracking_frame_;
  transform_odom_tracking.transform = ToTransform(state.nav_state_);
  tf2::doTransform(transform_tracking_base, transform_odom_base, transform_odom_tracking);
  transform_odom_base.child_frame_id = config.base_link_frame_;

  Eigen::Vector3d t_odom_base = ToVector3d(transform_odom_base.transform.translation);
  Eigen::Quaterniond R_odom_base = ToQuaterniond(transform_odom_base.transform.rotation);
  Eigen::Vector3d t_base_tracking = ToVector3d(transform_base_tracking.transform.translation);
  Eigen::Quaterniond R_base_tracking = ToQuaterniond(transform_base_tracking.transform.rotation);

  const Eigen::Vector3d& state_t = t_odom_base;
  const Eigen::Quaterniond& state_R = R_odom_base;
  const Eigen::Vector3d& state_omega = R_base_tracking * state.angular_velocity_tracking_frame_;
  Eigen::Matrix<double, 3, 3> state_omega_skrew;
  state_omega_skrew << 0.0, -state_omega[2], state_omega[1], state_omega[2], 0.0, -state_omega[0], -state_omega[1],
      state_omega[0], 0.0;
  Eigen::Vector3d state_v = R_base_tracking * state.nav_state_.bodyVelocity() + state_omega_skrew * (-t_base_tracking);

  out_msg.header.frame_id = config.odom_frame_;
  out_msg.header.stamp = result_time;
  out_msg.pose.pose = ToPose(state_t, state_R);
  out_msg.twist.twist = ToTwist(state_v, state_omega);
  out_msg.child_frame_id = config.base_link_frame_;
  return true;
}

#endif  // ODOM_FUSION_ROS_UTILS_H_
