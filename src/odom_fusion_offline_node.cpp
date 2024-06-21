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
#include "odom_fusion/debug_logger.h"
#include "odom_fusion/ros_utils.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "ros/callback_queue.h"
#include "rosgraph_msgs/Clock.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "urdf/model.h"
#include <tf2/utils.h>
#include <tf2/convert.h>

namespace
{
std::vector<geometry_msgs::TransformStamped> ReadStaticTransformsFromUrdf(const std::string& urdf_filename,
                                                                          tf2_ros::Buffer* const tf_buffer)
{
  std::cout << "Reading urdf " << urdf_filename << std::endl;
  urdf::Model model;
  if (!model.initFile(urdf_filename))
  {
    ROS_ERROR("Cannot read urdf file.");
  }
  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);
  std::vector<geometry_msgs::TransformStamped> transforms;
  for (const auto& link : links)
  {
    if (!link->getParent() || link->parent_joint->type != urdf::Joint::FIXED)
    {
      continue;
    }
    const urdf::Pose& pose = link->parent_joint->parent_to_joint_origin_transform;
    geometry_msgs::TransformStamped transform;
    transform.transform = ToTransform(ToVector3d(pose.position), ToQuaterniond(pose.rotation));
    transform.child_frame_id = link->name;
    transform.header.frame_id = link->getParent()->name;
    tf_buffer->setTransform(transform, "urdf", true);
    transforms.push_back(transform);
  }
  return transforms;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle nh;

  std::string bag_filename;
  std::string urdf_filename;
  if (argc < 3)
  {
    ROS_ERROR("Number of arguments < 2. Provide rosbag and urdf with file path.");
    return 1;
  }
  else
  {
    bag_filename = argv[1];
    urdf_filename = argv[2];
  }

  OdomFusionConfig config;
  LoadParams(config, nh);
  OdomFusionOfflineConfig config_offline;
  LoadParams(config_offline, nh);
  DebugLogger logger(config_offline.logging_filename);
  std::vector<sensor_msgs::Imu> imu_data;
  std::vector<nav_msgs::Odometry> odom_data;
  std::vector<nav_msgs::Odometry> reference_data;
  LoadBag(bag_filename, config_offline, imu_data, odom_data, reference_data);
  tf2_ros::Buffer tf_buffer;
  const auto current_urdf_transforms = ReadStaticTransformsFromUrdf(urdf_filename, &tf_buffer);
  tf_buffer.setUsingDedicatedThread(true);

  ros::Time t_init = imu_data.front().header.stamp;

  std::unique_ptr<OdomFusion> odom_fusion;
  if (config_offline.use_reference_orientation)
  {
    Eigen::Vector3d prior_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond prior_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d prior_velocity = Eigen::Vector3d::Zero();
    if (!reference_data.empty())
    {
      prior_position = ToVector3d(reference_data.front().pose.pose.position);
      prior_velocity = ToVector3d(reference_data.front().twist.twist.linear);
      prior_rotation = ToQuaterniond(reference_data.front().pose.pose.orientation);
      geometry_msgs::TransformStamped transform_odom_base;
      transform_odom_base.transform = ToTransform(prior_position, prior_rotation);

      geometry_msgs::TransformStamped transform_base_tracking;
      try
      {
        transform_base_tracking =
            tf_buffer.lookupTransform(config.base_link_frame_, config.tracking_frame_, ros::Time(0));
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
      }

      geometry_msgs::TransformStamped transform_odom_tracking;
      tf2::doTransform(transform_base_tracking, transform_odom_tracking, transform_odom_base);
      prior_position = ToVector3d(transform_odom_tracking.transform.translation);
      prior_rotation = ToQuaterniond(transform_odom_tracking.transform.rotation);
    }
    odom_fusion = std::make_unique<OdomFusion>(config, prior_position, prior_rotation, prior_velocity, t_init);
  }
  else {
    odom_fusion = std::make_unique<OdomFusion>(config);
  }
  size_t idx = 1;
  size_t observations_since_log = 0;
  size_t observations_since_print = 0;

  bool data_available = true;
  auto current_odom_data = odom_data.begin();
  auto current_fusedodom_data = reference_data.begin();
  auto current_imu_data = imu_data.begin();
  odom_fusion->AddOdomData(FromROSMsg(*current_odom_data));
  ++current_odom_data;
  odom_fusion->AddIMUData(FromROSMsg(*current_imu_data));
  ++current_imu_data;

  double dt = config.integration_dt;
  while (data_available)
  {
    ros::Time current_time = t_init + ros::Duration(double(idx) * dt);
    while (current_odom_data != odom_data.end() && std::prev(current_odom_data)->header.stamp <= current_time)
    {
      OdometryData odom_data_in_tracking;
      if (TransformOdomDataToOdomFrame(current_odom_data, tf_buffer, config, odom_data_in_tracking))
      {
        odom_fusion->AddOdomData(odom_data_in_tracking);
      }
      ++current_odom_data;
    }
    if (current_odom_data == odom_data.end())
    {
      data_available = false;
    }

    while (current_imu_data != imu_data.end() && std::prev(current_imu_data)->header.stamp <= current_time)
    {
      if (config.tracking_frame_ != current_imu_data->header.frame_id)
      {
        ROS_ERROR("IMU and Tracking frame are expected to be equal. IMU frame: %s Tracking frame: %s",
                  current_imu_data->header.frame_id.c_str(), config.tracking_frame_.c_str());
      }
      odom_fusion->AddIMUData(FromROSMsg(*current_imu_data));
      ++current_imu_data;
    }
    if (current_imu_data == imu_data.end())
      data_available = false;

    while (current_fusedodom_data != reference_data.end() &&
           std::next(current_fusedodom_data) != reference_data.end() &&
           current_fusedodom_data->header.stamp <= current_time)
    {
      ++current_fusedodom_data;
    }

    bool current_fusedodom_data_is_valid = false;
    if(!reference_data.empty()) {
      double t_delta = std::abs((current_fusedodom_data->header.stamp - current_time).toSec());
      current_fusedodom_data_is_valid = t_delta < dt / 2.0;
    }

    if (!odom_fusion->IsInitialized())
    {
      ROS_INFO_ONCE("Waiting for IMU data to initialize...");
      continue;
    }

    ros::Time result_time = odom_fusion->FullSensorDataUntil();

    if (result_time == ros::Time(0))
    {
      ROS_INFO("Waiting for odom data...");
      continue;
    }
    if (result_time == odom_fusion->GetTimeState())
    {
      continue;
    }
    if (!odom_fusion->Optimize(result_time))
    {
      continue;
    }

    if (current_fusedodom_data_is_valid || reference_data.empty())
    {
      if (config_offline.print_downsampling > 0 && observations_since_print >= config_offline.print_downsampling)
      {
        observations_since_print = 0;
        std::cout << "x " << odom_fusion->GetState().nav_state_.pose().x() << "\ty "
                  << odom_fusion->GetState().nav_state_.pose().y() << std::endl;
      }
      if (config_offline.log_downsampling > 0 && observations_since_log >= config_offline.log_downsampling)
      {
        geometry_msgs::TransformStamped transform_dummy;
        nav_msgs::Odometry odom_base_frame;
        TransformStateToBaseFrame(odom_fusion->GetState(), tf_buffer, config, result_time, transform_dummy,
                                  odom_base_frame);
        if (!reference_data.empty())
        {
            gtsam::NavState current_state_in_base_frame = ToNavState(odom_base_frame);
            logger.AddEntry(double(idx) * dt, current_state_in_base_frame, *std::prev(current_odom_data),
                            *current_fusedodom_data, odom_fusion->GetPoseCovariance(),
                            odom_fusion->GetVelocityCovariance(), odom_fusion->GetBiasCovariance(),
                            odom_fusion->GetBias(), odom_fusion->ZeroVelocityUpdateMode());

        }
        else
        {
          logger.AddEntry(double(idx) * dt, odom_fusion->GetState().nav_state_, *std::prev(current_odom_data),
                          *std::prev(current_odom_data), odom_fusion->GetPoseCovariance(),
                          odom_fusion->GetVelocityCovariance(), odom_fusion->GetBiasCovariance(),
                          odom_fusion->GetBias(), odom_fusion->ZeroVelocityUpdateMode());

        }
        observations_since_log = 0;
      }
    }
    observations_since_log++;
    observations_since_print++;

    ++idx;
  }
}
