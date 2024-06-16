//
// Created by aaa on 24-6-16.
//

#ifndef ANTS_PATHS_FOLLOWER_LINER_SPEED_REGULATOR_H
#define ANTS_PATHS_FOLLOWER_LINER_SPEED_REGULATOR_H
#include "ants_paths_follower/Array.h"
#include "ants_paths_follower/cubic_spline.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <queue>
#include <time.h>
// #include "quadprog/QuadProg++.h"
#include "common_private_msgs/planning_info.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "math.h"
#include <array>
#include <fstream>
#include <iterator>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <vector>
// #include <ants_local_planner/cubic_spline.h>

#include <std_msgs/Float64.h>

#include <std_msgs/Float32.h>
// #include <proto/fm.pb.h>
#include <fstream>

//longitude pid struct
struct pidKit {
    double error_sum;
    double last_error;
    ros::Time last_time;
    double last_speed;
};

class LinerSpeedRegulator {
public:
    LinerSpeedRegulator();
    void spin();

private:
    bool add_lateral_check(common_private_msgs::vehicle_status &last_front);
    bool add_yaw_check(common_private_msgs::vehicle_status &last_front);
    bool add_check(common_private_msgs::vehicle_status &last_front);
    void formationCallback(const geometry_msgs::Point::ConstPtr &formation_msg);
    void ReconfigurationCallback(const geometry_msgs::Point::ConstPtr &reconstruction_msg);
    bool GetVehicleInfo(int index, common_private_msgs::vehicle_status &out);
    void ClearpList();
    void Platoon0Callback(const nav_msgs::Odometry::ConstPtr &p_msg);
    void Platoon1Callback(const nav_msgs::Odometry::ConstPtr &p_msg);
    void Platoon2Callback(const nav_msgs::Odometry::ConstPtr &p_msg);
    void Platoon3Callback(const nav_msgs::Odometry::ConstPtr &p_msg);
    void Platoon4Callback(const nav_msgs::Odometry::ConstPtr &p_msg);
    void refTrajCallback(const common_private_msgs::planning_info::ConstPtr &trajectory_msg);
    void GapPlot(double error);
    void ModifyHeadway();
    double speed_control_1(pidKit &kit1);
    double speed_control_2(pidKit &kit2);
    double calc_speed(int mode, pidKit &kit1, pidKit &kit2);
    int findMatchpoint(double x, double y);
    void EM(pidKit &kit1, pidKit &kit2);


private:
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    ros::Subscriber formation_sub;
    ros::Subscriber reconfiguration_sub;
    ros::Subscriber platoon0_sub;
    ros::Subscriber platoon1_sub;
    ros::Subscriber platoon2_sub;
    ros::Subscriber platoon3_sub;
    ros::Subscriber platoon4_sub;
    ros::Subscriber mypathpose;


    // launch info
    int ID;
    double headway, current_headway, time_headway;
    double vmin, vmax, wmin, wmax;// 分别为v和w的控制量最小值最大值
    double overall_length;
    double emlike_hz;
    double Kp, Ki, Kd;
    double max_deceleration, max_acceleration;
    double Kp2, Ki2, Kd2;// for other formation to use
    double wheel_base;
    double vy = 0.0;
    double vx;
    double cf, cr, m, a, b, Iz;
    double forecast_time;
    double Q1, Q2, Q3, Q4, lqrR;
    int formation_type;
    int controller_type = 1;
    double Kp1, Kd1;// controller2

    // joint function
    int match_index;
    // reconfiguration function
    std::vector<std::pair<int, nav_msgs::Odometry>> pList;// 存储五个机器人的位置和速度
    int reconfiguration_flag;                             // 0-no, 1-add, 2-quit, 3-the add one. 4-after the add one one
    int last_reconfiguration_flag;
    ros::Time tick1;// quit start clock
    double reconfiguration_duration;
    double add_lateral_threshold, add_yaw_threshold;
    std::deque<int> add_keeper;

    // ros related
    common_private_msgs::vehicle_status front_pos;
    common_private_msgs::vehicle_status leader_pos;
    common_private_msgs::vehicle_status now_pos;
    CSRefPath refTrajectory;

    // for rviz
    nav_msgs::Path Trajectory;
    ros::Publisher Vis_traj;

    ros::Publisher cmd_pub;
    ros::Publisher cmd_pub2;
    // debug related
    ros::Publisher gap_plot_pub;
    std::ofstream tum_pose;

    pidKit kit1{0.0, 0.0, ros::Time::now(), 0.0};
    pidKit kit2{0.0, 0.0, ros::Time::now(), 0.0};
};
#endif//ANTS_PATHS_FOLLOWER_LINER_SPEED_REGULATOR_H
