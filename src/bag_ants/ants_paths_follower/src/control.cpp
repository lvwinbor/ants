#include "control.h"
#include "ros/init.h"
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>

//TODO:目前先不考虑特定角度摆臂不动的问题

namespace
{
auto motion_adjust_start_time_ = std::chrono::steady_clock::now();
auto motion_adjust_current_time_ = std::chrono::steady_clock::now();
auto front_arm_adjust_start_time_ = std::chrono::steady_clock::now();
auto rear_arm_adjust_start_time_ = std::chrono::steady_clock::now();
auto front_arm_adjust_current_time_ = std::chrono::steady_clock::now();
auto rear_arm_adjust_current_time_ = std::chrono::steady_clock::now();

int lastFrontArmAngle_{0};
int lastRearArmAngle_{0};
std::unique_ptr<ros::NodeHandle> nh_{nullptr};
std::unique_ptr<ros::NodeHandle> nhPrivate_{nullptr};
std::unique_ptr<ros::Publisher> commandPub_{nullptr};
std::shared_ptr<const common_private_msgs::autonomyMessage> motionStatus_{
    nullptr};
std::shared_ptr<const geometry_msgs::Pose2D> pose_{nullptr};
common_private_msgs::autonomyMessage command_{};

double yawKP_;
double yawErrorThreshold_;
int yawStabilityTime_;
double pointDistanceKP_;
double pointDistanceErrorThreshold_;
int pointDistanceStabilityTime_;
double lineDistanceKP_;
double lineDistanceErrorThreshold_;
int lineDistanceStabilityTime_;
double maxYawVelocity_;
double maxLinearVelocity_;

std::unique_ptr<ros::Rate> rate_{nullptr};
} // namespace

namespace
{
bool isAdjustFinished_(
    const double error,
    const double errorThreshold,
    const int stabilityTime,
    std::chrono::time_point<std::chrono::steady_clock> &startTime =
        motion_adjust_start_time_,
    std::chrono::time_point<std::chrono::steady_clock> &currentTime =
        motion_adjust_current_time_);
void brake_();
Eigen::Vector2d getYawUnitVector_(const double yaw);
void yawAdjust_(const double yawTarget);
void pointDistanceAdjust_(const double pointDistanceTarget);
void lineDistanceAdjust_(const double lineDistanceTarget);
void wallDistanceAdjust_(const double wallDistanceTarget);
double velocityLimit_(const double velocity, const double maxVelocity);

} // namespace

namespace
{

bool isAdjustFinished_(
    const double error,
    const double errorThreshold,
    const int stabilityTime,
    std::chrono::time_point<std::chrono::steady_clock> &startTime,
    std::chrono::time_point<std::chrono::steady_clock> &currentTime)
{
    if (abs(error) > errorThreshold)
    {
        startTime = std::chrono::steady_clock::now();
        return false; // 如果误差大于阈值，则认为没有调整完成
    }
    //为了防止误差一开始就小于阈值时，startTime_还未更新过
    if (startTime == std::chrono::steady_clock::time_point{})
    {
        startTime = std::chrono::steady_clock::now();
    }
    currentTime = std::chrono::steady_clock::now();
    auto errorTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime - startTime);
    std::cout << errorTime.count() << std::endl;

    if (errorTime >= std::chrono::milliseconds{stabilityTime})
    {
        startTime =
            std::chrono::steady_clock::time_point{}; // 将 startTime_ 重置
        return true;
    }
    else
    {
        return false;
    }
}
void brake_()
{
    while (ros::ok())
    {
        ros::spinOnce();
        command_.linearVelocity = 0.0f;
        command_.yawVelocity = 0.0f;
        commandPub_->publish(command_);
        rate_->sleep();
        if (isAdjustFinished_(motionStatus_->linearVelocity, 0.0, 500))
        {
            std::cout << "brake finished" << std::endl;
            break;
        }
    }
}
// 获取yaw方向的单位向量
Eigen::Vector2d getYawUnitVector_(const double yaw)
{
    // 创建一个二维单位向量
    Eigen::Vector2d unitVector;
    unitVector << cos(yaw), sin(yaw);
    return unitVector; // 返回单位向量
}

void yawAdjust_(const double yawTarget)
{
    while (ros::ok())
    {
        ros::spinOnce();
        double error = yawTarget - pose_->theta;
        std::cout << "yaw error: " << error << std::endl;
        command_.linearVelocity = 0.0f;
        command_.yawVelocity =
            static_cast<float>(velocityLimit_(yawKP_ * error, maxYawVelocity_));
        commandPub_->publish(command_);
        rate_->sleep();
        if (isAdjustFinished_(error, yawErrorThreshold_, yawStabilityTime_))
        {
            std::cout << "yawalign finished" << std::endl;
            break;
        }
    }
    brake_();
}

void pointDistanceAdjust_(const double xTarget, const double yTarget)
{
    while (ros::ok())
    {
        ros::spinOnce();
        Eigen::Vector2d distanceVector{xTarget - pose_->x, yTarget - pose_->y};
        Eigen::Vector2d vehicleYawUnitVector = getYawUnitVector_(pose_->theta);
        double pointDistanceError = std::sqrt(std::pow(pose_->x - xTarget, 2) +
                                              std::pow(pose_->y - yTarget, 2));
        if (distanceVector.dot(vehicleYawUnitVector) < 0)
        {
            pointDistanceError = -pointDistanceError;
        }
        double yawError =
            std::atan2(yTarget - pose_->y, xTarget - pose_->x) - pose_->theta;
        std::cout << "point distance error: " << pointDistanceError
                  << std::endl;
        std::cout << "yaw error: " << yawError << std::endl;
        command_.linearVelocity = static_cast<float>(velocityLimit_(
            pointDistanceKP_ * pointDistanceError, maxLinearVelocity_));
        command_.yawVelocity = static_cast<float>(
            velocityLimit_(yawKP_ * yawError, maxYawVelocity_));
        commandPub_->publish(command_);
        rate_->sleep();
        if (isAdjustFinished_(pointDistanceError,
                              pointDistanceErrorThreshold_,
                              pointDistanceStabilityTime_))
        {
            std::cout << "point distance align finished" << std::endl;
            break;
        }
    }
    brake_();
}
void lineDistanceAdjust_(
    const std::shared_ptr<const geometry_msgs::Pose2D> targetPoint)
{
    Eigen::Vector2d targetYawUnitVector = getYawUnitVector_(targetPoint->theta);
    while (ros::ok())
    {
        ros::spinOnce();
        Eigen::Vector2d distanceVector{targetPoint->x - pose_->x,
                                       targetPoint->y - pose_->y};
        double projectionDistance = targetYawUnitVector.dot(distanceVector);
        double error = projectionDistance;
        std::cout << "line distance error: " << error << std::endl;
        command_.yawVelocity = 0.0f;
        command_.linearVelocity = static_cast<float>(
            velocityLimit_(lineDistanceKP_ * error, maxLinearVelocity_));
        commandPub_->publish(command_);
        rate_->sleep();
        if (isAdjustFinished_(
                error, lineDistanceErrorThreshold_, lineDistanceStabilityTime_))
        {
            std::cout << "line distance align finished" << std::endl;
            break;
        }
    }
    brake_();
}
double velocityLimit_(const double velocity, const double maxVelocity)
{
    if (velocity > maxVelocity)
    {
        return maxVelocity;
    }
    else if (velocity < -maxVelocity)
    {
        return -maxVelocity;
    }
    else
    {
        return velocity;
    }
}
} // namespace

namespace control
{

void initializeControlParameter(
    std::shared_ptr<const common_private_msgs::autonomyMessage> motionStatus,
    std::shared_ptr<const geometry_msgs::Pose2D> pose)
{
    nh_ = std::make_unique<ros::NodeHandle>();
    nhPrivate_ = std::make_unique<ros::NodeHandle>("~");
    commandPub_ = std::make_unique<ros::Publisher>(
        nh_->advertise<common_private_msgs::autonomyMessage>("autonomyCommand",
                                                             1));
    motionStatus_ = motionStatus;
    pose_ = pose;
    rate_ = std::make_unique<ros::Rate>(100);

    yawKP_ = nhPrivate_->param("yawKP", 10.0);
    yawErrorThreshold_ = nhPrivate_->param("yawErrorThreshold", 0.002);
    yawStabilityTime_ = nhPrivate_->param("yawStabilityTime", 500);
    pointDistanceKP_ = nhPrivate_->param("pointDistanceKP", 0.5);
    pointDistanceErrorThreshold_ =
        nhPrivate_->param("pointDistanceErrorThreshold", 0.10);
    pointDistanceStabilityTime_ =
        nhPrivate_->param("pointDistanceStabilityTime", 500);
    lineDistanceKP_ = nhPrivate_->param("lineDistanceKP", 0.5);
    lineDistanceErrorThreshold_ =
        nhPrivate_->param("lineDistanceErrorThreshold", 0.01);
    lineDistanceStabilityTime_ =
        nhPrivate_->param("lineDistanceStabilityTime", 500);
}
void getToTargetPoint(
    const std::shared_ptr<const geometry_msgs::Pose2D> targetPoint,
    const bool isClimbing,
    const bool adjustYaw)
{
    if (isClimbing)
    {
        maxYawVelocity_ = nhPrivate_->param("maxObliqueYawVelocity", 1);
        maxLinearVelocity_ = nhPrivate_->param("maxObliqueLinearVelocity", 0.5);
    }
    else
    {
        maxYawVelocity_ = nhPrivate_->param("maxLevelYawVelocity", 2);
        maxLinearVelocity_ = nhPrivate_->param("maxLevelLinearVelocity", 1);
    }
    if (adjustYaw)
    {
        yawAdjust_(
            std::atan2(targetPoint->y - pose_->y, targetPoint->x - pose_->x));
        pointDistanceAdjust_(targetPoint->x, targetPoint->y);
        yawAdjust_(targetPoint->theta);
        lineDistanceAdjust_(targetPoint);
    }
    else
    {
        lineDistanceAdjust_(targetPoint);
    }
}
void swingToTargetAngle(const int targetFrontArmAngle,
                        const int targetRearArmAngle)
{
    command_.frontArmAngle = static_cast<int16_t>(targetFrontArmAngle);
    command_.rearArmAngle = static_cast<int16_t>(targetRearArmAngle);
    while (ros::ok())
    {
        ros::spinOnce();
        if (isAdjustFinished_(lastFrontArmAngle_ - motionStatus_->frontArmAngle,
                              0.0,
                              2000,
                              front_arm_adjust_start_time_,
                              front_arm_adjust_current_time_))
        {
            auto frontArmAngleError =
                targetFrontArmAngle - motionStatus_->frontArmAngle;
            if (abs(frontArmAngleError) <= 3 && frontArmAngleError != 0)
            {
                command_.frontArmAngle = static_cast<int16_t>(
                    targetFrontArmAngle + frontArmAngleError);
            }
        }
        if (isAdjustFinished_(lastRearArmAngle_ - motionStatus_->rearArmAngle,
                              0.0,
                              2000,
                              rear_arm_adjust_start_time_,
                              rear_arm_adjust_current_time_))
        {
            auto rearArmAngleError =
                targetRearArmAngle - motionStatus_->rearArmAngle;
            if (abs(rearArmAngleError) <= 3 && rearArmAngleError != 0)
            {
                command_.rearArmAngle = static_cast<int16_t>(
                    targetRearArmAngle + rearArmAngleError);
            }
        }
        if (motionStatus_->frontArmAngle == targetFrontArmAngle &&
            motionStatus_->rearArmAngle == targetRearArmAngle)
        {
            break;
        }
        lastFrontArmAngle_ = motionStatus_->frontArmAngle;
        lastRearArmAngle_ = motionStatus_->rearArmAngle;
        commandPub_->publish(command_);
        rate_->sleep();
    }
}

void goStraight(const double velocity)
{
    command_.linearVelocity = static_cast<float>(velocity);
    command_.yawVelocity = 0.0f;
    while (ros::ok())
    {
        commandPub_->publish(command_);
        rate_->sleep();
    }
}

} // namespace control