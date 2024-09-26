#include <Eigen/Dense>
#include <cmath>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <common_private_msgs/Pose3D.h>
#include <nav_msgs/Odometry.h>
#include <deque>
const double PI = M_PI;
const double line_distance = 0.5;
std::unique_ptr<geometry_msgs::Pose2D> target_pose{nullptr};
Eigen::Vector2d vertical_direction_vector;
Eigen::Vector2d parallel_direction_vector;
std_msgs::Float64 distance_to_right_wall;
std_msgs::Float64 distance_to_left_wall;
std_msgs::Float64 distance_to_front_wall;
sensor_msgs::PointCloud2 output_right;
sensor_msgs::PointCloud2 output_left;
sensor_msgs::PointCloud2 output_front;
common_private_msgs::Pose3D pose;
std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> front_cloud_deque;
const size_t max_frames = 20; // 设置储存帧数的上限
void poseHandler(const nav_msgs::Odometry::ConstPtr &odomIn)
{
    pose.x = odomIn->pose.pose.position.x;
    pose.y = odomIn->pose.pose.position.y;
    pose.z = odomIn->pose.pose.position.z;
}

void stairCallback(const geometry_msgs::PoseStamped::ConstPtr &odomIn)
{
    if (target_pose == nullptr)
    {
        target_pose = std::make_unique<geometry_msgs::Pose2D>();

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomIn->pose.orientation;
        tf::Matrix3x3(
            tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
            .getRPY(roll, pitch, yaw);
        target_pose->theta = yaw;
        target_pose->x = odomIn->pose.position.x;
        target_pose->y = odomIn->pose.position.y;
        vertical_direction_vector << cos(yaw + PI / 2), sin(yaw + PI / 2);
        parallel_direction_vector << cos(yaw), sin(yaw);
    }
}

void wallSegment(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_segment,
                 sensor_msgs::PointCloud2 &output,
                 std_msgs::Float64 &distance_to_wall,
                 const std::string &wall_direction)
{
    // 使用RANSAC算法识别平面（墙壁）
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud_segment);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
        ROS_WARN("Could not estimate a planar model for the %s wall.",
                 wall_direction.c_str());
        return;
    }

    // 提取平面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_segment);
    extract.setIndices(inliers);
    extract.setNegative(false); // 提取平面部分
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*wall_cloud);

    // 将墙壁点云转换为 PointCloud2 并发布
    pcl::toROSMsg(*wall_cloud, output);
    // output.header.frame_id = "ant01/vehicle"; // 设置合适的frame_id

    // 提取平面的系数
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    // 计算原点到平面的距离
    double vehicle_to_wall_distance =
        std::abs(a * pose.x + b * pose.y + c * pose.z + d) /
        std::sqrt(a * a + b * b + c * c);
    distance_to_wall.data = vehicle_to_wall_distance;
    ROS_INFO("Distance to the %s wall: %f meters",
             wall_direction.c_str(),
             distance_to_wall.data);
}
void processWallSegment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_segment,
    sensor_msgs::PointCloud2 &output,
    std_msgs::Float64 &distance_to_wall,
    const std::string &wall_direction)
{
    // 使用RANSAC算法识别平面（墙壁）
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud_segment);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
        ROS_WARN("Could not estimate a planar model for the %s wall.",
                 wall_direction.c_str());
        return;
    }

    // 提取平面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_segment);
    extract.setIndices(inliers);
    extract.setNegative(false); // 提取平面部分
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*wall_cloud);

    // 将墙壁点云转换为 PointCloud2 并发布
    pcl::toROSMsg(*wall_cloud, output);
    // output.header.frame_id = "ant01/vehicle"; // 设置合适的frame_id

    // 提取平面的系数
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    // 计算原点到平面的距离
    distance_to_wall.data =
        static_cast<float>(std::abs(d) / std::sqrt(a * a + b * b + c * c));

    ROS_INFO("Distance to the %s wall: %f meters",
             wall_direction.c_str(),
             distance_to_wall.data);
}
void frontCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if (target_pose != nullptr)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 将当前帧点云加入队列
        if (front_cloud_deque.size() >= max_frames)
        {
            front_cloud_deque.pop_front(); // 如果队列已满，移除最早的一帧
        }
        front_cloud_deque.push_back(cloud);

        // 合并多帧点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &stored_cloud : front_cloud_deque)
        {
            *merged_cloud += *stored_cloud;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr front_side_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        // 根据点的坐标进行过滤
        for (const auto &point : merged_cloud->points)
        {
            Eigen::Vector2d point_vector(point.x - target_pose->x,
                                         point.y - target_pose->y);

            double vertical_length =
                point_vector.dot(vertical_direction_vector);
            double parallel_length =
                point_vector.dot(parallel_direction_vector);

            if (std::abs(vertical_length) <= line_distance / 2.0 &&
                parallel_length >= 1 && point.z <= 4.0 && point.z >= 2.0)
            {
                front_side_cloud->points.push_back(point);
            }
        }

        // 识别前方的墙壁
        wallSegment(
            front_side_cloud, output_front, distance_to_front_wall, "front");
        output_front.header.frame_id = "map";
    }
}
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // 创建一个PCL点云对象指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 用于存放不同方向的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_side_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_side_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 根据点的坐标进行过滤
    for (const auto &point : cloud->points)
    {
        if (point.y < 0) // 右侧
        {
            right_side_cloud->points.push_back(point);
        }
        if (point.y > 0) // 左侧
        {
            left_side_cloud->points.push_back(point);
        }
    }

    // 识别右边的墙壁
    processWallSegment(
        right_side_cloud, output_right, distance_to_right_wall, "right");

    // 识别左边的墙壁
    processWallSegment(
        left_side_cloud, output_left, distance_to_left_wall, "left");
    output_right.header.frame_id = "ant01/vehicle";
    output_left.header.frame_id = "ant01/vehicle";
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "wall_distance_calculator");
    ros::NodeHandle nh;

    // 创建订阅者来订阅PointCloud2类型的消息
    ros::Subscriber sub =
        nh.subscribe("cloud_registered_body", 1, pointCloudCallback);
    ros::Subscriber sub_stair =
        nh.subscribe("pub_stairs_location_raw", 1, stairCallback);
    ros::Subscriber sub_cloud =
        nh.subscribe("registered_scan", 1, frontCloudCallback);
    ros::Subscriber odo_sub = nh.subscribe("state_estimation", 1, &poseHandler);

    ros::Publisher right_wall_cloud_pub =
        nh.advertise<sensor_msgs::PointCloud2>("right_wall_cloud", 1);
    ros::Publisher right_wall_distance_pub =
        nh.advertise<std_msgs::Float64>("right_wall_distance", 1);
    ros::Publisher left_wall_cloud_pub =
        nh.advertise<sensor_msgs::PointCloud2>("left_wall_cloud", 1);
    ros::Publisher left_wall_distance_pub =
        nh.advertise<std_msgs::Float64>("left_wall_distance", 1);
    ros::Publisher front_wall_cloud_pub =
        nh.advertise<sensor_msgs::PointCloud2>("front_wall_cloud", 1);
    ros::Publisher front_wall_distance_pub =
        nh.advertise<std_msgs::Float64>("front_wall_distance", 1);
    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        right_wall_distance_pub.publish(distance_to_right_wall);
        right_wall_cloud_pub.publish(output_right);
        left_wall_distance_pub.publish(distance_to_left_wall);
        left_wall_cloud_pub.publish(output_left);
        front_wall_distance_pub.publish(distance_to_front_wall);
        front_wall_cloud_pub.publish(output_front);
        rate.sleep();
    }

    return 0;
}
