#ifndef CONTROL_H
#define CONTROL_H
#include <Eigen/Dense>
#include <cmath>
#include <common_private_msgs/autonomyMessage.h>
#include <cstdlib>
#include <geometry_msgs/Pose2D.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
namespace control
{
void initializeControlParameter(
    std::shared_ptr<const common_private_msgs::autonomyMessage> motionStatus,
    std::shared_ptr<const geometry_msgs::Pose2D> pose);
void getToTargetPoint(
    const std::shared_ptr<const geometry_msgs::Pose2D> targetPoint,
    const bool isClimbing,
    const bool adjustYaw);
void swingToTargetAngle(const int targetFrontArmAngle,
                        const int targetRearArmAngle);
void goStraight(const double velocity);
} // namespace control

#endif