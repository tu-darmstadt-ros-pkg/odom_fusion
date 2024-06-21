#include "odom_fusion/odom_fusion.h"
#include <gtest/gtest.h>

TEST(DefaultTests, InitializationTest)
{
  OdomFusionConfig default_config;
  OdomFusion odom_fusion(default_config);
  ImuData imu_stationary = {
    ros::Time(0), { 0.0, 0.0, 9.80511 }, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()
  };
  OdometryData odom_stationary = { ros::Time(0), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  EXPECT_FALSE(odom_fusion.IsInitialized());
  odom_fusion.AddIMUData(imu_stationary);
  EXPECT_TRUE(odom_fusion.IsInitialized());
  gtsam::NavState state = odom_fusion.GetState().nav_state_;
  EXPECT_NEAR(state.t().norm(), 0.0, 1E-12);
  EXPECT_NEAR(state.quaternion().angularDistance(gtsam::Quaternion::Identity()), 0.0, 1E-12);
}

TEST(DefaultTests, ConstantLinearVelocityTest)
{
  OdomFusionConfig default_config;
  OdomFusion odom_fusion(default_config, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d{ 1.0, 0.0, 0.0 }, ros::Time(0));
  ImuData imu_stationary = {
    ros::Time(0), { 0.0, 0.0, 9.80511 }, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()
  };
  OdometryData odom_const_velocity = {
    ros::Time(0), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), { 1.0, 0.0, 0.0 }, Eigen::Vector3d::Zero()
  };
  double dt = 1E-2;
  size_t i_max = (size_t)std::round(5.0 / dt) + 1;
  for (size_t i = 0; i < i_max; ++i)
  {
    double current_time_s = (double)i * dt;
    ros::Time current_time = ros::Time(current_time_s);
    imu_stationary.time = current_time;
    odom_const_velocity.time = current_time;
    odom_fusion.AddIMUData(imu_stationary);
    if (i % 2 == 0)
    {
      odom_fusion.AddOdomData(odom_const_velocity);
    }
    if (i % 10 == 0 && i > 0)
    {
      odom_fusion.Optimize(current_time);
      gtsam::NavState state = odom_fusion.GetState().nav_state_;
      EXPECT_NEAR(state.t().x(), current_time_s, 1E-7);
      EXPECT_NEAR(state.t().y(), 0.0, 1E-7);
      EXPECT_NEAR(state.t().z(), 0.0, 1E-7);
      EXPECT_NEAR(state.v().x(), 1.0, 1E-7);
      EXPECT_NEAR(state.v().y(), 0.0, 1E-7);
      EXPECT_NEAR(state.v().z(), 0.0, 1E-7);
      EXPECT_NEAR(state.quaternion().angularDistance(gtsam::Quaternion::Identity()), 0.0, 1E-12);
    }
  }
}

TEST(DefaultTests, ConstantLinearAccelerationTest)
{
  OdomFusionConfig default_config;
  OdomFusion odom_fusion(default_config, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d::Zero(), ros::Time(0));
  ImuData imu_stationary = {
    ros::Time(0), { 1.0, 0.0, 9.80511 }, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()
  };
  OdometryData odom_const_velocity = { ros::Time(0), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  double dt = 1E-2;
  size_t i_max = (size_t)std::round(5.0 / dt) + 1;
  for (size_t i = 0; i < i_max; ++i)
  {
    double current_time_s = (double)i * dt;
    ros::Time current_time = ros::Time(current_time_s);
    imu_stationary.time = current_time;
    odom_const_velocity.time = current_time;
    odom_const_velocity.linear_velocity.x() = current_time_s;
    odom_fusion.AddIMUData(imu_stationary);
    if (i % 2 == 0)
    {
      odom_fusion.AddOdomData(odom_const_velocity);
    }
    if (i % 10 == 0 && i > 0)
    {
      odom_fusion.Optimize(current_time);
      gtsam::NavState state = odom_fusion.GetState().nav_state_;
      double estimated_t_x = current_time_s * current_time_s / 2.0;
      EXPECT_NEAR(state.t().x(), estimated_t_x, 1E-7);
      EXPECT_NEAR(state.t().y(), 0.0, 1E-7);
      EXPECT_NEAR(state.t().z(), 0.0, 1E-7);
      EXPECT_NEAR(state.v().x(), current_time_s, 1E-7);
      EXPECT_NEAR(state.v().y(), 0.0, 1E-7);
      EXPECT_NEAR(state.v().z(), 0.0, 1E-7);
      EXPECT_NEAR(state.quaternion().angularDistance(gtsam::Quaternion::Identity()), 0.0, 1E-12);
    }
  }
}

TEST(DefaultTests, ConstantAngularVelocityTest)
{
  OdomFusionConfig default_config;
  OdomFusion odom_fusion(default_config, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d::Zero(), ros::Time(0));
  double angular_rate = 0.5;
  ImuData imu_observation = {
    ros::Time(0), { 0.0, 0.0, 9.80511 }, { 0.0, 0.0, angular_rate }, Eigen::Quaterniond::Identity()
  };
  OdometryData odom_stationary = { ros::Time(0), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  double dt = 1E-2;
  size_t i_max = (size_t)std::round(5.0 / dt) + 1;
  for (size_t i = 0; i < i_max; ++i)
  {
    double current_time_s = (double)i * dt;
    ros::Time current_time = ros::Time(current_time_s);
    imu_observation.time = current_time;
    odom_stationary.time = current_time;

    double gt_rotation_angle = current_time_s * angular_rate;
    double qw = std::cos(gt_rotation_angle / 2.0);
    double qz = std::sin(gt_rotation_angle / 2.0);
    auto reference_orientation = Eigen::Quaterniond(qw, 0.0, 0.0, qz);
    odom_fusion.AddIMUData(imu_observation);

    if (i % 2 == 0)
    {
      odom_fusion.AddOdomData(odom_stationary);
    }
    if (i % 10 == 0 && i > 0)
    {
      odom_fusion.Optimize(current_time);
      gtsam::NavState state = odom_fusion.GetState().nav_state_;
      EXPECT_NEAR(state.t().x(), 0.0, 1E-7);
      EXPECT_NEAR(state.t().y(), 0.0, 1E-7);
      EXPECT_NEAR(state.t().z(), 0.0, 1E-7);
      EXPECT_NEAR(state.v().x(), 0.0, 1E-7);
      EXPECT_NEAR(state.v().y(), 0.0, 1E-7);
      EXPECT_NEAR(state.v().z(), 0.0, 1E-7);
      EXPECT_NEAR(state.quaternion().angularDistance(reference_orientation), 0.0, 1E-7);
    }
  }
}

TEST(DefaultTests, CircleTrajectoryTest)
{
  OdomFusionConfig default_config;
  double angular_rate = M_PI / 10.0;
  double linear_velocity = M_PI / 10.0;
  OdomFusion odom_fusion(default_config, { 0.0, -1.0, 0.0 }, Eigen::Quaterniond::Identity(),
                         { linear_velocity, 0.0, 0.0 }, ros::Time(0));
  double radius = linear_velocity  / angular_rate;
  double centripetal_acceleration = linear_velocity * linear_velocity / radius;
  ImuData imu_observation = {
    ros::Time(0), { 0.0, centripetal_acceleration, 9.80511 }, { 0.0, 0.0, angular_rate }, Eigen::Quaterniond::Identity()
  };
  OdometryData odom_const_velocity = { ros::Time(0),
                                       Eigen::Vector3d::Zero(),
                                       Eigen::Quaterniond::Identity(),
                                       { linear_velocity, 0.0, 0.0 },
                                       Eigen::Vector3d::Zero() };
  double dt = 1E-2;
  size_t i_max = (size_t)std::round(20.0 / dt) + 1;

  for (size_t i = 0; i < i_max; ++i)
  {
    double current_time_s = (double)i * dt;
    ros::Time current_time = ros::Time(current_time_s);
    imu_observation.time = current_time;
    odom_const_velocity.time = current_time;
    double gt_rotation_angle = current_time_s * angular_rate;
    double qw = std::cos(gt_rotation_angle / 2.0);
    double qz = std::sin(gt_rotation_angle / 2.0);
    double px = std::sin(gt_rotation_angle);
    double py = -std::cos(gt_rotation_angle);
    auto reference_orientation = Eigen::Quaterniond(qw, 0.0, 0.0, qz);
    odom_fusion.AddIMUData(imu_observation);
    if (i % 1 == 0)
    {
      odom_fusion.AddOdomData(odom_const_velocity);
    }
    if (i % 2 == 0 && i > 0)
    {
      odom_fusion.Optimize(current_time);
      gtsam::NavState state = odom_fusion.GetState().nav_state_;
      EXPECT_NEAR(state.t().x(), px, 0.1);
      EXPECT_NEAR(state.t().y(), py, 0.1);
      EXPECT_NEAR(state.t().z(), 0.0, 0.1);
      EXPECT_NEAR(state.bodyVelocity().x(), linear_velocity, 0.0001);
      EXPECT_NEAR(state.bodyVelocity().y(), 0.0, 0.0001);
      EXPECT_NEAR(state.bodyVelocity().z(), 0.0, 0.0001);
      EXPECT_NEAR(state.quaternion().angularDistance(reference_orientation), 0.0, 0.0005);      
    }    
  }
}




TEST(DefaultTests, StaticAngularBiasCompensationTest)
{
  OdomFusionConfig default_config;
  default_config.prior_bias_sigma = 0.1 * default_config.prior_bias_sigma;
  OdomFusion odom_fusion(default_config, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d{ 1.0, 0.0, 0.0 }, ros::Time(0));
  ImuData imu_observation_biased = {
    ros::Time(0), { 0.0, 0.0, 9.80511 }, { 0.01, 0.0, 0.0 }, Eigen::Quaterniond::Identity()
  };
  OdometryData odom_observation_static = {
    ros::Time(0), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
  };
  double dt = 1E-2;
  size_t i_max = (size_t)std::round(5.0 / dt) + 1;
  for (size_t i = 0; i < i_max; ++i)
  {
    double current_time_s = (double)i * dt;
    ros::Time current_time = ros::Time(current_time_s);
    imu_observation_biased.time = current_time;
    odom_observation_static.time = current_time;
    odom_fusion.AddIMUData(imu_observation_biased);
    if (i % 2 == 0)
    {
      odom_fusion.AddOdomData(odom_observation_static);
    }
    if (i % 10 == 0 && i > 0)
    {
      odom_fusion.Optimize(current_time);
      gtsam::NavState state = odom_fusion.GetState().nav_state_;
      // EXPECT_NEAR(state.t().x(), 0.0, 1E-5);
      // EXPECT_NEAR(state.t().y(), 0.0, 1E-5);
      // EXPECT_NEAR(state.t().z(), 0.0, 1E-5);
      // EXPECT_NEAR(state.v().x(), 0.0, 1E-5);
      // EXPECT_NEAR(state.v().y(), 0.0, 1E-5);
      // EXPECT_NEAR(state.v().z(), 0.0, 1E-5);
      // EXPECT_NEAR(state.quaternion().angularDistance(gtsam::Quaternion::Identity()), 0.0, 1E-5);
      printf("deltaq: %f ", state.quaternion().angularDistance(gtsam::Quaternion::Identity()));
      gtsam::imuBias::ConstantBias bias = odom_fusion.GetBias();
      printf("bias: %f, %f, %f, %f, %f, %f\n", bias.accelerometer().x(), bias.accelerometer().y(), bias.accelerometer().z(), bias.gyroscope().x(), bias.gyroscope().y(), bias.gyroscope().z());
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}