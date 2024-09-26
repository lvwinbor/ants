#include "control.h"
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>

std::unique_ptr<ros::NodeHandle> BaseControl::nh_{nullptr};
std::unique_ptr<ros::NodeHandle> BaseControl::nh_private_{nullptr};
std::unique_ptr<ros::Rate> BaseControl::rate_{nullptr};
std::unique_ptr<common_private_msgs::Pose3D> BaseControl::pose{nullptr};
std::unique_ptr<common_private_msgs::autonomyMessage>
    BaseControl::motion_status{nullptr};
std::unique_ptr<common_private_msgs::autonomyMessage> BaseControl::command{
    nullptr};

Timer::Timer(int timer_num)
: timer_num_(timer_num)
{
    time_pairs_.resize(timer_num_);
    for (auto time_pair_ : time_pairs_)
    {
        time_pair_.first = std::chrono::steady_clock::now();
        time_pair_.second = std::chrono::steady_clock::now();
    }
}

bool Timer::isFinished(const double error,
                       const double error_threshold,
                       const int stability_time,
                       const int timer_identifier)
{
    int vector_index = timer_identifier - 1;
    if (abs(error) > error_threshold)
    {
        time_pairs_[vector_index].first = std::chrono::steady_clock::now();
        return false; // 如果误差大于阈值，则认为没有调整完成
    }
    //为了防止误差一开始就小于阈值时，startTime_还未更新过
    if (time_pairs_[vector_index].first ==
        std::chrono::steady_clock::time_point{})
    {
        time_pairs_[vector_index].first = std::chrono::steady_clock::now();
    }
    time_pairs_[vector_index].second = std::chrono::steady_clock::now();
    auto errorTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        time_pairs_[vector_index].second - time_pairs_[vector_index].first);
    std::cout << errorTime.count() << std::endl;

    if (errorTime >= std::chrono::milliseconds{stability_time})
    {
        time_pairs_[vector_index].first =
            std::chrono::steady_clock::time_point{}; // 将 startTime_ 重置
        return true;
    }
    else
    {
        return false;
    }
}

void BaseControl::check_ptr()
{
    if (nh_ == nullptr || nh_private_ == nullptr || rate_ == nullptr ||
        command == nullptr)
    {
        std::cerr << "BaseControl initialization failed!" << std::endl;
        exit(5);
    }
    while (ros::ok())
    {
        if (pose == nullptr || motion_status == nullptr)
        {
            std::cout << "ptr in BaseControl is null" << std::endl;
            ros::spinOnce();
            rate_->sleep();
        }
        else
        {
            break;
        }
    }
}
void MessageUpdate::messageUpdate()
{
    while (ros::ok())
    {
        ros::spinOnce();
        if (command != nullptr)
        {
            command->yaw_speed =
                speedLimit_(command->yaw_speed, max_yaw_speed_);
            command->linear_speed =
                speedLimit_(command->linear_speed, max_linear_speed_);
            command_pub_.publish(*command);
        }
        else
        {
            std::cout << "command is null" << std::endl;
        }
        rate_->sleep();
    }
}

void MessageUpdate::poseHandler_(const nav_msgs::Odometry::ConstPtr &odomIn)
{
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geometry_quaternion =
        odomIn->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geometry_quaternion.x,
                                 geometry_quaternion.y,
                                 geometry_quaternion.z,
                                 geometry_quaternion.w))
        .getRPY(roll, pitch, yaw);
    if (pose == nullptr)
    {
        pose = std::make_unique<common_private_msgs::Pose3D>();
    }
    pose->x = odomIn->pose.pose.position.x - cos(yaw) * sensor_offset_x_ +
              sin(yaw) * sensor_offset_y_;
    pose->y = odomIn->pose.pose.position.y - sin(yaw) * sensor_offset_x_ -
              cos(yaw) * sensor_offset_y_;
    pose->z = odomIn->pose.pose.position.z;
    pose->yaw = yaw;
    pose->pitch = pitch;
    pose->roll = roll;
}

void MessageUpdate::motionStatusHandler_(
    const common_private_msgs::autonomyMessage::ConstPtr &status)
{
    if (motion_status == nullptr)
    {
        motion_status =
            std::make_unique<common_private_msgs::autonomyMessage>();
    }
    motion_status->front_arm_angle = status->front_arm_angle;
    motion_status->rear_arm_angle = status->rear_arm_angle;
    motion_status->linear_speed = status->linear_speed;
    motion_status->yaw_speed = status->yaw_speed;
}

double MessageUpdate::speedLimit_(const double speed, const double max_speed)
{
    if (speed > max_speed)
    {
        return max_speed;
    }
    else if (speed < -max_speed)
    {
        return -max_speed;
    }
    else
    {
        return speed;
    }
}

void YawControl::control(const double yaw_target)
{
    double yaw = yaw_normalization(yaw_target);
    check_ptr();
    while (ros::ok())
    {
        double error = yaw - pose->yaw;
        error = yaw_normalization(error);
        std::cout << "yaw error: " << error << std::endl;
        command->linear_speed = 0.0;
        command->yaw_speed = kp_ * error;
        rate_->sleep();
        if (timer_.isFinished(error, error_threshold_, stability_time_))
        {
            std::cout << "yaw align finished" << std::endl;
            break;
        }
    }
}
double YawControl::yaw_normalization(double yaw)
{
    while (yaw > PI || yaw < -PI)
    {
        if (yaw > PI)
        {
            yaw -= 2 * PI;
        }
        else
        {
            yaw += 2 * PI;
        }
    }
    return yaw;
}
void PointDistanceControl::control(
    const std::shared_ptr<const common_private_msgs::Pose3D> target_point)
{
    check_ptr();
    while (ros::ok())
    {
        Eigen::Vector2d distance_vector{target_point->x - pose->x,
                                        target_point->y - pose->y};
        double distance_error = distance_vector.norm();
        Eigen::Vector2d vehicle_yaw_unit_vector{cos(pose->yaw), sin(pose->yaw)};
        if (distance_vector.dot(vehicle_yaw_unit_vector) < 0)
        {
            distance_error = -distance_error;
        }
        double yaw_error =
            std::atan2(distance_vector.y(), distance_vector.x()) - pose->yaw;
        std::cout << "point distance error: " << distance_error << std::endl;
        std::cout << "yaw error: " << yaw_error << std::endl;
        command->linear_speed = distance_error * kp_;
        command->yaw_speed = yaw_error * yaw_kp_in_point_distance_;
        rate_->sleep();
        if (timer_.isFinished(
                distance_error, error_threshold_, stability_time_))
        {
            std::cout << "point distance align finished" << std::endl;
            break;
        }
    }
}

void LineDistanceControl::control(
    const std::shared_ptr<const common_private_msgs::Pose3D> target_point)
{
    check_ptr();
    Eigen::Vector2d target_yaw_unit_vector{cos(target_point->yaw),
                                           sin(target_point->yaw)};
    while (ros::ok())
    {
        Eigen::Vector2d distance_vector{target_point->x - pose->x,
                                        target_point->y - pose->y};
        double distance_error = target_yaw_unit_vector.dot(distance_vector);
        std::cout << "line distance error: " << distance_error << std::endl;
        command->yaw_speed = 0.0;
        command->linear_speed = distance_error * kp_;
        rate_->sleep();
        if (timer_.isFinished(
                distance_error, error_threshold_, stability_time_))
        {
            std::cout << "line distance align finished" << std::endl;
            break;
        }
    }
}

void Brake::control()
{
    check_ptr();
    command->linear_speed = 0.0;
    command->yaw_speed = 0.0;
    while (ros::ok())
    {
        rate_->sleep();
        if (timer_.isFinished(motion_status->linear_speed +
                                  motion_status->yaw_speed,
                              0.0,
                              stability_time_))
        {
            std::cout << "brake finished" << std::endl;
            break;
        }
    }
}

void GoStraight::control(double speed, int time)
{
    check_ptr();
    while (ros::ok())
    {
        command->linear_speed = speed;
        command->yaw_speed = 0.0;
        if (timer_.isFinished(0, 0, time))
        {
            break;
        }
    }
    brake_.control();
}
void GetToTargetPoint::control(
    const std::shared_ptr<const common_private_msgs::Pose3D> target_point,
    const bool whether_control_yaw)
{
    if (whether_control_yaw)
    {
        yaw_control_.control(
            std::atan2(target_point->y - pose->y, target_point->x - pose->x));
        brake_.control();
        point_distance_control_.control(target_point);
        brake_.control();
        yaw_control_.control(target_point->yaw);
        brake_.control();
        line_distance_control_.control(target_point);
        brake_.control();
    }
    else
    {
        line_distance_control_.control(target_point);
        brake_.control();
    }
}

void SwingToTargetAngle::control(const int target_front_arm_angle,
                                 const int target_rear_arm_angle)
{
    check_ptr();
    command->front_arm_angle = static_cast<int16_t>(target_front_arm_angle);
    command->rear_arm_angle = static_cast<int16_t>(target_rear_arm_angle);
    while (ros::ok())
    {
        if (timer_.isFinished(last_front_arm_angle_ -
                                  motion_status->front_arm_angle,
                              0.0,
                              stability_time_,
                              1))
        {
            auto front_arm_angle_error =
                target_front_arm_angle - motion_status->front_arm_angle;
            if (abs(front_arm_angle_error) <= 3 && front_arm_angle_error != 0)
            {
                command->front_arm_angle = static_cast<int16_t>(
                    command->front_arm_angle + front_arm_angle_error);
            }
        }
        if (timer_.isFinished(last_rear_arm_angle_ -
                                  motion_status->rear_arm_angle,
                              0.0,
                              stability_time_,
                              2))
        {
            auto rear_arm_angle_error =
                target_rear_arm_angle - motion_status->rear_arm_angle;
            if (abs(rear_arm_angle_error) <= 3 && rear_arm_angle_error != 0)
            {
                command->rear_arm_angle = static_cast<int16_t>(
                    command->rear_arm_angle + rear_arm_angle_error);
            }
        }
        if (motion_status->front_arm_angle == target_front_arm_angle &&
            motion_status->rear_arm_angle == target_rear_arm_angle)
        {
            break;
        }
        last_front_arm_angle_ = motion_status->front_arm_angle;
        last_rear_arm_angle_ = motion_status->rear_arm_angle;
        rate_->sleep();
    }
}

void AttitudeAdjustment::upStairs(double target_distance)
{
    check_ptr();
    while (ros::ok())
    {
        ros::spinOnce();
        double wall_error = target_right_wall_distance_ - right_wall_->data;
        command->yaw_speed = kp_ * wall_error;
        command->linear_speed = linear_speed_;
        rate_->sleep();
        if (timer_.isFinished(front_wall_->data, target_distance, 100))
        {
            break;
        }
    }
    brake_.control();
}
void AttitudeAdjustment::downStairs(
    const std::shared_ptr<const common_private_msgs::Pose3D> target_point)
{
    check_ptr();
    Eigen::Vector2d target_yaw_unit_vector{cos(target_point->yaw),
                                           sin(target_point->yaw)};
    while (ros::ok())
    {
        ros::spinOnce();
        Eigen::Vector2d distance_vector{target_point->x - pose->x,
                                        target_point->y - pose->y};
        double distance_error = target_yaw_unit_vector.dot(distance_vector);
        std::cout << "line distance error: " << distance_error << std::endl;
        double wall_error = target_left_wall_distance_ - left_wall_->data;
        command->yaw_speed = -kp_ * wall_error;
        command->linear_speed = linear_speed_;
        rate_->sleep();
        if (timer_.isFinished(distance_error, downStairs_error_threshold_, 100))
        {
            break;
        }
    }
    brake_.control();
}
void AttitudeAdjustment::check_ptr()
{
    BaseControl::check_ptr();
    while (ros::ok())
    {
        if (right_wall_ == nullptr || left_wall_ == nullptr ||
            front_wall_ == nullptr)
        {
            std::cout << "ptr in AttitudeAdjustment is null" << std::endl;
            ros::spinOnce();
            rate_->sleep();
        }
        else
        {
            break;
        }
    }
}

void AttitudeAdjustment::rightWallHandler(
    const std_msgs::Float64::ConstPtr &msg)
{
    if (right_wall_ == nullptr)
    {
        right_wall_ = std::make_unique<std_msgs::Float64>();
    }
    right_wall_->data = msg->data;
}
void AttitudeAdjustment::leftWallHandler(const std_msgs::Float64::ConstPtr &msg)
{
    if (left_wall_ == nullptr)
    {
        left_wall_ = std::make_unique<std_msgs::Float64>();
    }
    left_wall_->data = msg->data;
}

void AttitudeAdjustment::frontWallHandler(
    const std_msgs::Float64::ConstPtr &msg)
{
    if (front_wall_ == nullptr)
    {
        front_wall_ = std::make_unique<std_msgs::Float64>();
    }
    front_wall_->data = msg->data;
}