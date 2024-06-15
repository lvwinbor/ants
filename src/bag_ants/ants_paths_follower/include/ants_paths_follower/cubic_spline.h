/*
三次样条插值曲线
*/
#ifndef CUBIC_SPLINE_PLANNER_H
#define CUBIC_SPLINE_PLANNER_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Eigen>
// #include <routing/Astar_searcher.h>
// #include <custom_messages/mapmodel.h>
#include <geometry_msgs/PoseArray.h>
// #include <custom_messages/planning_info.h>
#include "common_private_msgs/planning_info.h"
#include "common_private_msgs/vehicle_status.h"

#define PI 3.1415926

/*差分计算函数
输入：一列数
输出：输出数列为输入数列两两之间的差值
*/
std::vector<double> vec_diff(const std::vector<double> &input);

/*累加计算函数
输入：一列数
输出：输出数列为输入数列从开始到对应位置的累加
*/
std::vector<double> cum_sum(const std::vector<double> &input);

// 定义结构体
struct Point {
    double x;
    double y;
};
struct RefPath {
    double r_x;        // x坐标
    double r_y;        // y坐标
    double r_yaw;      // 航向角
    double r_curvature;// 曲率
    double r_s;        // 弧长
};
struct splineParameter {
    // 三次多项式的四个系数，注意a为最低次项，依次升高
    double a;
    double b;
    double c;
    double d;
};

/*三次样条曲线插值最终得到每个区间拟合的系数
  d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
*/
class Spline {
public:
    std::vector<Point> point;// x为横坐标，y为函数值坐标
    std::vector<splineParameter> parameter;
    int nx = 0;           // 区间数，即一共有几段要拟合的曲线
    std::vector<double> h;// x的步长
public:
    // 构造函数
    Spline() = default;
    Spline(const std::vector<double> &x_, const std::vector<double> &y_);

    // 求t位置的0阶导数
    double calc(double t);
    // 求t位置的1阶导数
    double calc_d(double t);
    // 求t位置的2阶导数
    double calc_dd(double t);

private:
    // 生成系数矩阵
    Eigen::MatrixXd calc_A();
    //生成解向量
    Eigen::VectorXd calc_B();
    //递归二分法找根
    int bisect(double t, int start, int end);
};
/*
 二维平面三次样条曲线
https://blog.csdn.net/u013468614/article/details/108416552
 */
class Spline2D {
private:
    Spline sx;            // x的样条曲线
    Spline sy;            // y的样条曲线
    std::vector<double> s;// 弧长，样条曲线的参数
public:
    std::vector<RefPath> path;

public:
    // 构造函数
    Spline2D() = default;
    //    Spline2D(const std::vector<double> &x, const std::vector<double> &y);
    explicit Spline2D(const std::vector<Point> &point);
    // 根据输入的弧长vector进行插值
    void interpolate(const std::vector<double> &s_vector);


private:
    // 位置计算函数
    Point calc_position(double s_t);
    // 曲率计算函数
    double calc_curvature(double s_t);
    // 航向角计算函数
    double calc_yaw(double s_t);
    // 弧长计算函数，输出数列为从起点到当前位置的弧长
    std::vector<double> calc_s(const std::vector<double> &x, const std::vector<double> &y);
};

struct SplineCarState {
    double x;
    double y;
    double yaw;
    double speed;
    double angular_speed;
    // SplineCarState(){};
    // SplineCarState(double x_, double y_, double yaw_, double speed_, double
    // angular_speed_):
    //     x(x_), y(y_), yaw(yaw_), speed(speed_), angular_speed(angular_speed_)
    // {}
};

class CSRefPath {
public:
public:
    std::vector<RefPath> r_path;

public:
    CSRefPath() = default;
    ~CSRefPath() = default;
    void CSclear();
    void load(Spline2D &spline, double overall_length);// 载入路径
    void joint(const CSRefPath &path);                 // 添加路径
    void copypath(const CSRefPath &path, int index);   // 从索引开始添加路径，索引从0开始

    CSRefPath &operator=(const CSRefPath &input);
};

// plannercore
class CubicSplineClass {
public:
    CubicSplineClass(ros::Publisher trajectory_pub);
    void FrontCallback(const nav_msgs::Odometry::ConstPtr &front_msg);
    void LeaderCallback(const nav_msgs::Odometry::ConstPtr &leader_msg);
    void SelfCallback(const nav_msgs::Odometry::ConstPtr &self_msg);
    // void ObsCallback(const custom_messages::mapmodel::ConstPtr& obs_msg);
    // bool CollisionCheck(const CSRefPath &r_path, const SplineCarState
    // &carstate, const AstarPathFinder* tmpmap, int mode); // mode=0,static map;
    // mode=1, static map and dynamic obstacle
    void planning();
    void PlatoonCallback1(const nav_msgs::Odometry::ConstPtr &p_msg);
    void PlatoonCallback2(const nav_msgs::Odometry::ConstPtr &p_msg);
    void PlatoonCallback3(const nav_msgs::Odometry::ConstPtr &p_msg);
    void PlatoonCallback4(const nav_msgs::Odometry::ConstPtr &p_msg);
    void PlatoonCallback5(const nav_msgs::Odometry::ConstPtr &p_msg);

    bool GetVehicleInfo(
            int mode, int index,
            geometry_msgs::PoseStamped &out);// mode=0:front, mode=1:leader
    bool GetVehiclemyInfo(int index, SplineCarState &out);
    void ReconfigurationCallback(
            const geometry_msgs::Point::ConstPtr &reconstruction_msg);
    // void UpdateKeeper(custom_messages::vehicle_status last_back);
    void UpdateKeeper(common_private_msgs::vehicle_status last_back);
    int findMatchpoint(double x, double y, CSRefPath &input_trajectory);
    bool add_check(SplineCarState &last_front);
    bool add_lateral_check(SplineCarState &last_front);
    bool add_yaw_check(SplineCarState &last_front);

    // formation change
    void update_formation(double formation_length_unit);

public:
    std::vector<SplineCarState> keeper;
    ros::Publisher _trajectory_pub;
    geometry_msgs::PoseStamped front;
    geometry_msgs::PoseStamped leader;
    double front_yaw;
    double leader_yaw;
    SplineCarState mystate;
    // custom_messages::mapmodel obstacle;
    bool self_initial = false;
    bool front_initial = false;
    bool leader_initial = false;

    // formation change vector
    std::vector<Eigen::Vector2d> formation_diamond;
    std::vector<Eigen::Vector2d> formation_triangle;
};

#endif