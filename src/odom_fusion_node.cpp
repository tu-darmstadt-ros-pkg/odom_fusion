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

#include "odom_fusion/odom_fusion_node.h"
#include "odom_fusion/odom_fusion.h"
#include "odom_fusion/ros_utils.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <memory>
//#include <tf/tf.h>

OdomFusionNode::OdomFusionNode(ros::NodeHandle &nh) : tf_listener_(tf_buffer_)
{
  LoadParams(config_, nh);
  odom_fusion_ = std::make_unique<OdomFusion>(config_);
  imu_subscriber_ =
      nh.subscribe("/imu/data", 10, &OdomFusionNode::ImuCallback, this, ros::TransportHints().tcpNoDelay(true));
  odom_subscriber_ =
      nh.subscribe("/odom_raw", 10, &OdomFusionNode::OdomCallback, this, ros::TransportHints().tcpNoDelay(true));
  syscommand_subscriber_ = nh.subscribe("/syscommand", 10, &OdomFusionNode::SyscommandCallback, this);
  fused_odom_publisher_ = nh.advertise<nav_msgs::Odometry>("odom_fused", 5);
  bias_publisher_ = nh.advertise<geometry_msgs::Twist>("bias_estimate", 5);
  timers_.push_back(nh.createTimer(ros::Duration(config_.integration_dt), &OdomFusionNode::PublishTF, this));
}

void OdomFusionNode::SyscommandCallback(const std_msgs::StringConstPtr& msg) {
  if (msg->data == "reset_cartographer") {
    ROS_INFO("Resetting now due to syscommand.");
    odom_fusion_ = std::make_unique<OdomFusion>(config_);
  }
}

void OdomFusionNode::ImuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  if (config_.tracking_frame_ != msg->header.frame_id)
  {
    ROS_ERROR("IMU and Tracking frame are expected to be equal. IMU frame: %s Tracking frame: %s",
              msg->header.frame_id.c_str(), config_.tracking_frame_.c_str());
  }
  odom_fusion_->AddIMUData(FromROSMsg(*msg));
}

void OdomFusionNode::OdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  OdometryData odom_data_in_tracking;
  if (TransformOdomDataToOdomFrame(msg, tf_buffer_, config_, odom_data_in_tracking))
  {
    odom_fusion_->AddOdomData(odom_data_in_tracking);
  }
}

void OdomFusionNode::PublishTF(const ros::TimerEvent &unused_timer_event)
{
  if (!odom_fusion_->IsInitialized())
  {
    ROS_INFO_ONCE("Waiting for IMU data to initialize...");
    return;
  }
  ros::Time result_time = odom_fusion_->FullSensorDataUntil();
  if(result_time == ros::Time(0)) {
    ROS_INFO("Waiting for odom data...");
    return;
  }
  if(result_time == odom_fusion_->GetTimeState()) {
    return;
  }
  if (!odom_fusion_->Optimize(result_time))
  {
    return;
  }

  geometry_msgs::TransformStamped transform_odom_base;
  nav_msgs::Odometry msg;
  if (TransformStateToBaseFrame(odom_fusion_->GetState(), tf_buffer_, config_, result_time, transform_odom_base, msg))
  {
    tf_broadcaster_.sendTransform(transform_odom_base);
    fused_odom_publisher_.publish(msg);
  }

  if (bias_publisher_.getNumSubscribers() > 0)
  {
    geometry_msgs::Twist bias_msg;
    const gtsam::imuBias::ConstantBias& bias = odom_fusion_->GetBias();
    bias_msg.linear.x = bias.accelerometer().x();
    bias_msg.linear.y = bias.accelerometer().y();
    bias_msg.linear.z = bias.accelerometer().z();
    bias_msg.angular.x = bias.gyroscope().x();
    bias_msg.angular.y = bias.gyroscope().y();
    bias_msg.angular.z = bias.gyroscope().z();
    bias_publisher_.publish(bias_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle nh;
  OdomFusionNode odom_fusion_node(nh);
  while (ros::ok())
  {
    ros::spin();
  }
  return 0;
}
