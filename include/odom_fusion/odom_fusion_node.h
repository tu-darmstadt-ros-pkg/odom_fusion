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

#ifndef ODOM_FUSION_ODOM_FUSION_NODE_H_
#define ODOM_FUSION_ODOM_FUSION_NODE_H_
#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2/convert.h>

#include "odom_fusion/odom_fusion.h"

class OdomFusionNode
{
public:
  explicit OdomFusionNode(ros::NodeHandle& nh);

  void SyscommandCallback(const std_msgs::StringConstPtr& msg);

  void ImuCallback(const sensor_msgs::ImuConstPtr& msg);

  void OdomCallback(const nav_msgs::OdometryConstPtr& measurement);

  void PublishTF(const ros::TimerEvent& unused_timer_event);

private:
  OdomFusionConfig config_;

  ros::Subscriber imu_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber syscommand_subscriber_;
  ros::Publisher fused_odom_publisher_;
  ros::Publisher bias_publisher_;
  std::vector<ros::Timer> timers_;
  std::unique_ptr<OdomFusion> odom_fusion_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

#endif  // ODOM_FUSION_ODOM_FUSION_NODE_H_
