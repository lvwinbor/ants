#ifndef CONTROL_H
#define CONTROL_H
#include <Eigen/Dense>
#include <cmath>
#include <common_private_msgs/autonomyMessage.h>
#include <cstdlib>
#include <common_private_msgs/Pose3D.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>
#include <utility>
#include <std_msgs/Float64.h>
#include <vector>
const double PI = M_PI;
class Timer
{
private:
    int timer_num_;
    std::vector<std::pair<std::chrono::time_point<std::chrono::steady_clock>,
                          std::chrono::time_point<std::chrono::steady_clock>>>
        time_pairs_;

public:
    Timer() = delete;
    explicit Timer(int timer_num);

    bool isFinished(const double error,
                    const double error_threshold,
                    const int stability_time,
                    const int timer_identifier = 1);
};
class BaseControl
{
protected:
    static std::unique_ptr<ros::NodeHandle> nh_;
    static std::unique_ptr<ros::NodeHandle> nh_private_;
    static std::unique_ptr<ros::Rate> rate_;

public:
    static std::unique_ptr<common_private_msgs::Pose3D> pose;
    static std::unique_ptr<common_private_msgs::autonomyMessage> motion_status;
    static std::unique_ptr<common_private_msgs::autonomyMessage> command;

protected:
    BaseControl()
    {
        if (nh_ == nullptr)
        {
            nh_ = std::make_unique<ros::NodeHandle>();
            nh_private_ = std::make_unique<ros::NodeHandle>("~");
            rate_ = std::make_unique<ros::Rate>(100);
            command = std::make_unique<common_private_msgs::autonomyMessage>();
            command->front_arm_angle =
                nh_private_->param<int>("init_front_arm_angle", -110);
            command->rear_arm_angle =
                nh_private_->param<int>("init_rear_arm_angle", -110);
        }
    }
    virtual ~BaseControl() = default;
    virtual void check_ptr();
};

class MessageUpdate : public BaseControl
{
    ros::Publisher command_pub_{
        nh_->advertise<common_private_msgs::autonomyMessage>("autonomy_command",
                                                             1)};
    ros::Subscriber pose_sub_{nh_->subscribe<nav_msgs::Odometry>(
        "state_estimation", 1, &MessageUpdate::poseHandler_, this)};
    ros::Subscriber motion_status_sub_{
        nh_->subscribe<common_private_msgs::autonomyMessage>(
            "motion_status", 1, &MessageUpdate::motionStatusHandler_, this)};
    const double sensor_offset_x_{
        nh_private_->param<double>("sensor_offset_x", 0.205)};
    const double sensor_offset_y_{
        nh_private_->param<double>("sensor_offset_y", 0.0)};
    const double max_yaw_speed_{nh_private_->param<double>("max_yaw_speed", 1)};
    const double max_linear_speed_{
        nh_private_->param<double>("max_linear_speed", 0.5)};

public:
    MessageUpdate() = default;

    void messageUpdate();

private:
    void poseHandler_(const nav_msgs::Odometry::ConstPtr &odomIn);

    void motionStatusHandler_(
        const common_private_msgs::autonomyMessage::ConstPtr &status);

    double speedLimit_(const double speed, const double max_speed);
};

class MotionControl : public BaseControl
{
protected:
    double kp_;
    double error_threshold_;
    int stability_time_;

    Timer timer_{1};

protected:
    MotionControl() = delete;
    MotionControl(const double kp,
                  const double error_threshold,
                  const int stability_time)
    : kp_{kp}
    , error_threshold_{error_threshold}
    , stability_time_{stability_time}
    {
    }
};

class YawControl : public MotionControl
{
public:
    YawControl()
    : MotionControl{nh_private_->param<double>("yaw_kp", 0.0),
                    nh_private_->param<double>("yaw_error_threshold", 0.0),
                    nh_private_->param<int>("yaw_stability_time", 0)}
    {
    }

    void control(const double yaw_target);

private:
    double yaw_normalization(double yaw);
};
class PointDistanceControl : public MotionControl
{
private:
    double yaw_kp_in_point_distance_{
        nh_private_->param<double>("yaw_kp_in_point_distance", 0.0)};

public:
    PointDistanceControl()
    : MotionControl{
          nh_private_->param<double>("point_distance_kp", 0.0),
          nh_private_->param<double>("point_distance_error_threshold", 0.0),
          nh_private_->param<int>("point_distance_stability_time", 0)}
    {
    }

    void control(
        const std::shared_ptr<const common_private_msgs::Pose3D> target_point);
};
class LineDistanceControl : public MotionControl
{
public:
    LineDistanceControl()
    : MotionControl{nh_private_->param<double>("line_distance_kp", 0.0),
                    nh_private_->param("line_distance_error_threshold", 0.0),
                    nh_private_->param<int>("line_distance_stability_time", 0)}
    {
    }

    void control(
        const std::shared_ptr<const common_private_msgs::Pose3D> target_point);
};
class Brake : public BaseControl
{
private:
    int stability_time_{nh_private_->param<int>("brake_stability_time", 500)};
    Timer timer_{1};

public:
    Brake() = default;

    void control();
};
class GoStraight : public BaseControl
{
private:
    Timer timer_{1};
    Brake brake_{};

public:
    GoStraight() = default;
    void control(double speed, int time);
};

class GetToTargetPoint : public BaseControl
{
private:
    YawControl yaw_control_{};
    PointDistanceControl point_distance_control_{};
    LineDistanceControl line_distance_control_{};
    Brake brake_{};

public:
    GetToTargetPoint() = default;

    void control(
        const std::shared_ptr<const common_private_msgs::Pose3D> target_point,
        const bool whether_control_yaw = true);
};

class SwingToTargetAngle : public BaseControl
{
private:
    int last_front_arm_angle_{0};
    int last_rear_arm_angle_{0};

    int stability_time_{nh_private_->param<int>("swing_stability_time", 4000)};
    Timer timer_{2};

public:
    SwingToTargetAngle() = default;

    void control(const int target_front_arm_angle,
                 const int target_rear_arm_angle);
};

class AttitudeAdjustment : public BaseControl
{
private:
    const double kp_{
        nh_private_->param<double>("attitude_adjustment_kp", 20.0)};
    const double linear_speed_{nh_private_->param<double>("linear_speed", 0.1)};
    const double target_right_wall_distance_{
        nh_private_->param<double>("target_right_wall_distance", 10.0)};
    const double target_left_wall_distance_{
        nh_private_->param<double>("target_left_wall_distance", 10.0)};
    const double downStairs_error_threshold_{
        nh_private_->param<double>("down_stairs_error_threshold", 0.1)};
    Brake brake_{};
    Timer timer_{1};

    std::unique_ptr<std_msgs::Float64> right_wall_{nullptr};
    std::unique_ptr<std_msgs::Float64> left_wall_{nullptr};
    std::unique_ptr<std_msgs::Float64> front_wall_{nullptr};
    ros::Subscriber right_wall_distance_sub_{nh_->subscribe<std_msgs::Float64>(
        "right_wall_distance", 1, &AttitudeAdjustment::rightWallHandler, this)};
    ros::Subscriber left_wall_distance_sub_{nh_->subscribe<std_msgs::Float64>(
        "left_wall_distance", 1, &AttitudeAdjustment::leftWallHandler, this)};
    ros::Subscriber front_wall_distance_sub_{nh_->subscribe<std_msgs::Float64>(
        "front_wall_distance", 1, &AttitudeAdjustment::frontWallHandler, this)};

public:
    AttitudeAdjustment() = default;
    void upStairs(double target_distance);
    void downStairs(
        const std::shared_ptr<const common_private_msgs::Pose3D> target_point);
    void check_ptr() override;

private:
    void rightWallHandler(const std_msgs::Float64::ConstPtr &msg);

    void leftWallHandler(const std_msgs::Float64::ConstPtr &msg);
    void frontWallHandler(const std_msgs::Float64::ConstPtr &msg);
};

#endif