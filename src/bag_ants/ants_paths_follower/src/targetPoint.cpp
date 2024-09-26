#include "control.h"
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
std::shared_ptr<common_private_msgs::Pose3D> upstairs_first_target_point{
    nullptr};
std::shared_ptr<common_private_msgs::Pose3D> upstairs_second_target_point{
    nullptr};
std::shared_ptr<common_private_msgs::Pose3D> upstairs_third_target_point{
    nullptr};
std::shared_ptr<common_private_msgs::Pose3D> upstairs_fourth_target_point{
    nullptr};
std::shared_ptr<common_private_msgs::Pose3D> downstairs_first_target_point{
    nullptr};
std::shared_ptr<common_private_msgs::Pose3D> downstairs_second_target_point{
    nullptr};
std::shared_ptr<common_private_msgs::Pose3D> downstairs_third_target_point{
    nullptr};
double upstairs_second_distance = -0.42;
double upstairs_third_distance = -0.2;
double upstairs_fourth_distance = 0.04;
// double downstairs_first_distance = -0.2;
// double downstairs_second_distance = -0.05;
// double downstairs_third_distance = 0.2;
double downstairs_second_distance = 0.30;
double downstairs_third_distance = 0.50;
void getDownstairsFirstTargetPoint(
    const std::unique_ptr<common_private_msgs::Pose3D> &current_pose)
{
    Eigen::Vector2d direction(cos(current_pose->yaw), sin(current_pose->yaw));
    // 计算点到第一点的向量
    Eigen::Vector2d vec(current_pose->x - upstairs_second_target_point->x,
                        current_pose->y - upstairs_second_target_point->y);

    // 计算投影
    double dot_product = vec.dot(direction);
    double length_squared = direction.dot(direction);
    double projection_scale = dot_product / length_squared;

    // 计算投影点
    Eigen::Vector2d projection_point =
        Eigen::Vector2d(upstairs_second_target_point->x,
                        upstairs_second_target_point->y) +
        projection_scale * direction;
    downstairs_first_target_point =
        std::make_shared<common_private_msgs::Pose3D>();
    downstairs_first_target_point->x = projection_point.x();
    downstairs_first_target_point->y = projection_point.y();
    downstairs_first_target_point->z = current_pose->z;
    downstairs_first_target_point->pitch = current_pose->pitch;
    downstairs_first_target_point->roll = current_pose->roll;
    downstairs_first_target_point->yaw = upstairs_second_target_point->yaw + PI;

    downstairs_second_target_point =
        std::make_shared<common_private_msgs::Pose3D>();
    downstairs_second_target_point->x =
        downstairs_first_target_point->x +
        cos(downstairs_first_target_point->yaw) * downstairs_second_distance;
    downstairs_second_target_point->y =
        downstairs_first_target_point->y +
        sin(downstairs_first_target_point->yaw) * downstairs_second_distance;
    downstairs_second_target_point->z = downstairs_first_target_point->z;
    downstairs_second_target_point->pitch =
        downstairs_first_target_point->pitch;
    downstairs_second_target_point->roll = downstairs_first_target_point->roll;
    downstairs_second_target_point->yaw = downstairs_first_target_point->yaw;

    downstairs_third_target_point =
        std::make_shared<common_private_msgs::Pose3D>();
    downstairs_third_target_point->x =
        downstairs_first_target_point->x +
        cos(downstairs_first_target_point->yaw) * downstairs_third_distance;
    downstairs_third_target_point->y =
        downstairs_first_target_point->y +
        sin(downstairs_first_target_point->yaw) * downstairs_third_distance;
    downstairs_third_target_point->z = downstairs_first_target_point->z;
    downstairs_third_target_point->pitch = downstairs_first_target_point->pitch;
    downstairs_third_target_point->roll = downstairs_first_target_point->roll;
    downstairs_third_target_point->yaw = downstairs_first_target_point->yaw;
}
void rawStairHandler(const geometry_msgs::PoseStamped::ConstPtr &odomIn)
{
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odomIn->pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
        .getRPY(roll, pitch, yaw);
    if (upstairs_second_target_point == nullptr)
    {
        upstairs_second_target_point =
            std::make_shared<common_private_msgs::Pose3D>();

        upstairs_second_target_point->yaw = yaw;
        upstairs_second_target_point->pitch = pitch;
        upstairs_second_target_point->roll = roll;
        upstairs_second_target_point->x =
            odomIn->pose.position.x + cos(yaw) * upstairs_second_distance;
        upstairs_second_target_point->y =
            odomIn->pose.position.y + sin(yaw) * upstairs_second_distance;
        upstairs_second_target_point->z = odomIn->pose.position.z;
    }
    if (upstairs_third_target_point == nullptr)
    {
        upstairs_third_target_point =
            std::make_shared<common_private_msgs::Pose3D>();
        upstairs_third_target_point->yaw = yaw;
        upstairs_third_target_point->pitch = pitch;
        upstairs_third_target_point->roll = roll;
        upstairs_third_target_point->x =
            odomIn->pose.position.x + cos(yaw) * upstairs_third_distance;
        upstairs_third_target_point->y =
            odomIn->pose.position.y + sin(yaw) * upstairs_third_distance;
        upstairs_third_target_point->z = odomIn->pose.position.z;
    }
    if (upstairs_fourth_target_point == nullptr)
    {
        upstairs_fourth_target_point =
            std::make_shared<common_private_msgs::Pose3D>();
        upstairs_fourth_target_point->yaw = yaw;
        upstairs_fourth_target_point->pitch = pitch;
        upstairs_fourth_target_point->roll = roll;
        upstairs_fourth_target_point->x =
            odomIn->pose.position.x + cos(yaw) * upstairs_fourth_distance;
        upstairs_fourth_target_point->y =
            odomIn->pose.position.y + sin(yaw) * upstairs_fourth_distance;
        upstairs_fourth_target_point->z = odomIn->pose.position.z;
    }
}
// void downStairHandler(const geometry_msgs::PoseStamped::ConstPtr &odomIn)
// {
//     double roll, pitch, yaw;
//     geometry_msgs::Quaternion geoQuat = odomIn->pose.orientation;
//     tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
//         .getRPY(roll, pitch, yaw);
//     if (downstairs_first_target_point == nullptr)
//     {
//         downstairs_first_target_point =
//             std::make_shared<common_private_msgs::Pose3D>();
//         downstairs_first_target_point->yaw = yaw;
//         downstairs_first_target_point->pitch = pitch;
//         downstairs_first_target_point->roll = roll;
//         downstairs_first_target_point->x =
//             odomIn->pose.position.x + cos(yaw) * downstairs_first_distance;
//         downstairs_first_target_point->y =
//             odomIn->pose.position.y + sin(yaw) * downstairs_first_distance;
//         downstairs_first_target_point->z = odomIn->pose.position.z;
//     }
//     if (downstairs_second_target_point == nullptr)
//     {
//         downstairs_second_target_point =
//             std::make_shared<common_private_msgs::Pose3D>();
//         downstairs_second_target_point->yaw = yaw;
//         downstairs_second_target_point->pitch = pitch;
//         downstairs_second_target_point->roll = roll;
//         downstairs_second_target_point->x =
//             odomIn->pose.position.x + cos(yaw) * downstairs_second_distance;
//         downstairs_second_target_point->y =
//             odomIn->pose.position.y + sin(yaw) * downstairs_second_distance;
//         downstairs_second_target_point->z = odomIn->pose.position.z;
//     }
//     if (downstairs_third_target_point == nullptr)
//     {
//         downstairs_third_target_point =
//             std::make_shared<common_private_msgs::Pose3D>();
//         downstairs_third_target_point->yaw = yaw;
//         downstairs_third_target_point->pitch = pitch;
//         downstairs_third_target_point->roll = roll;
//         downstairs_third_target_point->x =
//             odomIn->pose.position.x + cos(yaw) * downstairs_third_distance;
//         downstairs_third_target_point->y =
//             odomIn->pose.position.y + sin(yaw) * downstairs_third_distance;
//         downstairs_third_target_point->z = odomIn->pose.position.z;
//     }
// }

void topicStairHandler(const geometry_msgs::PoseStamped::ConstPtr &odomIn)
{
    if (upstairs_first_target_point == nullptr)
    {
        upstairs_first_target_point =
            std::make_shared<common_private_msgs::Pose3D>();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomIn->pose.orientation;
        tf::Matrix3x3(
            tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
            .getRPY(roll, pitch, yaw);
        upstairs_first_target_point->yaw = yaw;
        upstairs_first_target_point->pitch = pitch;
        upstairs_first_target_point->roll = roll;
        upstairs_first_target_point->x = odomIn->pose.position.x;
        upstairs_first_target_point->y = odomIn->pose.position.y;
        upstairs_first_target_point->z = odomIn->pose.position.z;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_point");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    MessageUpdate message_update;
    std::thread t1(&MessageUpdate::messageUpdate, &message_update);

    ros::Subscriber topicStairSub = nh.subscribe<geometry_msgs::PoseStamped>(
        "pub_stairs_location_topic", 1, topicStairHandler);

    ros::Rate rate(100);
    while (ros::ok())
    {
        if (upstairs_first_target_point != nullptr)
        {
            break;
        }
        else
        {
            ros::spinOnce();
            std::cout << "upstairs first target point is null" << std::endl;
            rate.sleep();
        }
    }
    GoStraight go_straight;
    SwingToTargetAngle swing_to_target_angle;
    GetToTargetPoint get_to_target_point;
    AttitudeAdjustment attitude_adjustment;
    YawControl yaw_control;
    swing_to_target_angle.control(-108, -108);
    get_to_target_point.control(upstairs_first_target_point);

    ros::Subscriber raw_stair_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "pub_stairs_location_raw", 1, rawStairHandler);
    while (ros::ok())
    {
        if (upstairs_second_target_point != nullptr)
        {
            break;
        }
        else
        {
            ros::spinOnce();
            std::cout << "upstairs second target point is null" << std::endl;
            rate.sleep();
        }
    }
    get_to_target_point.control(upstairs_second_target_point);

    swing_to_target_angle.control(6, -25);
    get_to_target_point.control(upstairs_third_target_point, false);
    swing_to_target_angle.control(6, 0);
    get_to_target_point.control(upstairs_fourth_target_point, false);
    swing_to_target_angle.control(9, 12);
    attitude_adjustment.upStairs(1.60);

    swing_to_target_angle.control(45, 12);
    go_straight.control(0.1, 5000);
    getDownstairsFirstTargetPoint(BaseControl::pose);
    swing_to_target_angle.control(-108, -108);
    attitude_adjustment.upStairs(0.5);
    yaw_control.control(upstairs_second_target_point->yaw + PI);
    // ros::Subscriber down_stair_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    //     "pub_stairs_location_down", 1, downStairHandler);
    // while (ros::ok())
    // {
    //     if (downstairs_first_target_point != nullptr)
    //     {
    //         break;
    //     }
    //     else
    //     {
    //         ros::spinOnce();
    //         std::cout << "downstairs first target point is null" << std::endl;
    //         rate.sleep();
    //     }
    // }

    get_to_target_point.control(downstairs_first_target_point);
    get_to_target_point.control(downstairs_second_target_point, false);
    swing_to_target_angle.control(49, 0);
    get_to_target_point.control(downstairs_third_target_point, false);
    swing_to_target_angle.control(19, 0);
    std::shared_ptr<common_private_msgs::Pose3D>
        downstairs_fourth_target_point =
            std::make_shared<common_private_msgs::Pose3D>();
    *downstairs_fourth_target_point = *upstairs_fourth_target_point;
    downstairs_fourth_target_point->yaw += PI;
    attitude_adjustment.downStairs(downstairs_fourth_target_point);
    swing_to_target_angle.control(-3, 1);
    std::shared_ptr<common_private_msgs::Pose3D> downstairs_fifth_target_point =
        std::make_shared<common_private_msgs::Pose3D>();
    *downstairs_fifth_target_point = *upstairs_second_target_point;
    downstairs_fifth_target_point->yaw += PI;
    get_to_target_point.control(downstairs_fifth_target_point, false);
    swing_to_target_angle.control(-108, -108);
}
//distance 0.1m   -0.2m
//49
//前：19