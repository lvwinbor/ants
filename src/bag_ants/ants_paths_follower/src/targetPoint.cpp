#include "common_private_msgs/autonomyMessage.h"
#include "control.h"

#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

std::shared_ptr<geometry_msgs::Pose2D> firstTargetPoint{nullptr};
std::shared_ptr<geometry_msgs::Pose2D> secondTargetPoint{nullptr};
std::shared_ptr<geometry_msgs::Pose2D> thirdTargetPoint{nullptr};
std::shared_ptr<geometry_msgs::Pose2D> pose{nullptr};
std::shared_ptr<common_private_msgs::autonomyMessage> motionStatus{nullptr};
double secondDistance = -0.42;
double thirdDistance = 0.04;
const double sensorOffsetX = 0.205;
const double sensorOffsetY = 0.0;
void rawStairHandler(const geometry_msgs::PoseStamped::ConstPtr &odomIn)
{
    if (secondTargetPoint == nullptr)
    {
        secondTargetPoint = std::make_shared<geometry_msgs::Pose2D>();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomIn->pose.orientation;
        tf::Matrix3x3(
            tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
            .getRPY(roll, pitch, yaw);
        secondTargetPoint->theta = yaw;
        secondTargetPoint->x =
            odomIn->pose.position.x + cos(yaw) * secondDistance;
        secondTargetPoint->y =
            odomIn->pose.position.y + sin(yaw) * secondDistance;
    }
    if (thirdTargetPoint == nullptr)
    {
        thirdTargetPoint = std::make_shared<geometry_msgs::Pose2D>();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomIn->pose.orientation;
        tf::Matrix3x3(
            tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
            .getRPY(roll, pitch, yaw);
        thirdTargetPoint->theta = yaw;
        thirdTargetPoint->x =
            odomIn->pose.position.x + cos(yaw) * thirdDistance;
        thirdTargetPoint->y =
            odomIn->pose.position.y + sin(yaw) * thirdDistance;
    }
}

// void TargetPointOffset()

void topicStairHandler(const geometry_msgs::PoseStamped::ConstPtr &odomIn)
{
    if (firstTargetPoint == nullptr)
    {
        firstTargetPoint = std::make_shared<geometry_msgs::Pose2D>();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomIn->pose.orientation;
        tf::Matrix3x3(
            tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
            .getRPY(roll, pitch, yaw);
        firstTargetPoint->theta = yaw;
        firstTargetPoint->x = odomIn->pose.position.x;
        firstTargetPoint->y = odomIn->pose.position.y;
    }
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn)
{
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
        .getRPY(roll, pitch, yaw);
    if (pose == nullptr)
    {
        pose = std::make_shared<geometry_msgs::Pose2D>();
    }
    pose->x = static_cast<double>(odomIn->pose.pose.position.x -
                                  cos(yaw) * sensorOffsetX +
                                  sin(yaw) * sensorOffsetY);
    pose->y = static_cast<double>(odomIn->pose.pose.position.y -
                                  sin(yaw) * sensorOffsetX -
                                  cos(yaw) * sensorOffsetY);
    pose->theta = static_cast<double>(yaw);
}
void motionStatusHandler(
    const common_private_msgs::autonomyMessage::ConstPtr &status)
{
    if (motionStatus == nullptr)
    {
        motionStatus = std::make_shared<common_private_msgs::autonomyMessage>();
    }
    motionStatus->frontArmAngle = status->frontArmAngle;
    motionStatus->rearArmAngle = status->rearArmAngle;
    motionStatus->linearVelocity = status->linearVelocity;
    motionStatus->yawVelocity = status->yawVelocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "targetPoint");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    ros::Subscriber topicStairSub = nh.subscribe<geometry_msgs::PoseStamped>(
        "pub_stairs_location_topic", 1, topicStairHandler);
    ros::Subscriber odomSub =
        nh.subscribe<nav_msgs::Odometry>("state_estimation", 1, odomHandler);
    ros::Subscriber motionStatusSub =
        nh.subscribe<common_private_msgs::autonomyMessage>(
            "motionStatus", 1, motionStatusHandler);
    ros::Rate rate(100);
    while (firstTargetPoint == nullptr)
    {
        ros::spinOnce();
        rate.sleep();
    }
    while (motionStatus == nullptr)
    {
        ros::spinOnce();
        rate.sleep();
    }
    while (pose == nullptr)
    {
        ros::spinOnce();
        rate.sleep();
    }
    control::initializeControlParameter(motionStatus, pose);
    control::swingToTargetAngle(65, 65);
    control::getToTargetPoint(firstTargetPoint, false, true);
    ros::Subscriber rawStairSub = nh.subscribe<geometry_msgs::PoseStamped>(
        "pub_stairs_location_raw", 1, rawStairHandler);
    while (secondTargetPoint == nullptr)
    {
        ros::spinOnce();
        rate.sleep();
    }
    control::getToTargetPoint(secondTargetPoint, false, true);

    control::swingToTargetAngle(-8, -1);
    control::getToTargetPoint(thirdTargetPoint, true, false);

    control::swingToTargetAngle(-7, -15);
    control::goStraight(0.1);
    // control::swingToTargetAngle(const int targetFrontArmAngle, const int
    // targetRearArmAngle)
}