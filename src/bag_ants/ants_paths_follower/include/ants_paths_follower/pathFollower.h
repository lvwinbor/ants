#include "common_private_msgs/controlMessage.h"
#include <cmath>
#include <ros/ros.h>
#include <stdlib.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <common_private_msgs/controlMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


class PathFollower {
private://变量
    //ros相关变量
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;
    ros::Subscriber subGoal;
    ros::Subscriber subOdom;
    ros::Subscriber subPath;
    ros::Subscriber subSpeed;
    ros::Subscriber subStop;
    common_private_msgs::controlMessage command;
    //单例模式相关变量
    static PathFollower *_pathFollower;//这是单例对象，静态指针
    static std::mutex i_mutex;         //加锁

    const float PI = M_PI;

    const double sensorOffsetX;// 传感器在车体坐标系下的X值
    const double sensorOffsetY;// 传感器在车体坐标系下的Y值
    const int pubSkipNum;      //机器人路径上每隔几个点发布一次

    const bool twoWayDrive;      // 双向驱动
    const double lookAheadDis;   // 预瞄距离
    const double yawRateGain;    // 转向增益
    const double stopYawRateGain;// 停车时的转向增益
    const double maxYawRate;     // 最大转向速率
    const double maxSpeed;       // 最大速度
    const double maxAccel;       // 最大加速度
    const double switchTimeThre; // 切换方向的时间阈值
    const double dirDiffThre;    // 方向差阈值，小于该值时不转向
    const double stopDisThre;    // 停车距离阈值，小于该值时停车
    const double slowDwnDisThre; // 减速距离阈值，小于该值时减速
    const bool useInclRateToSlow;// 是否根据角速度来减速
    const double inclRateThre;   // 关于x和y的角速度阈值,单位为度
    const double slowRate1;      // 减速率1
    const double slowRate2;      // 减速率2
    const double slowTime1;      // 减速时间1
    const double slowTime2;      // 减速时间2
    const bool useInclToStop;    // 是否根据倾斜度来停车
    const double inclThre;       // roll或pitch的阈值,单位为度
    const double stopTime;       // 停车时间
    const bool noRotAtStop;      // 停车时是否不旋转
    const bool noRotAtGoal;      // 到达目标点时是否不旋转
    const double autonomySpeed;  // 自主模式下的速度
    const double joyToSpeedDelay;// 手柄输入到自主模式速度输入的最小间隔时间
    const double trackDisThre;   // 跟踪距离阈值
    const bool trackModeAble;    // 是否可以进入跟踪模式

    const std::string antname;

    double goalX = 0;// 目标点的X坐标
    double goalY = 0;// 目标点的Y坐标

    float targetVelocity = 0;//获取的速度
    int safetyStop = 0;      //安全停止信号的级别
    //车辆当前位姿
    float vehicleX = 0;            //车辆中心在世界坐标系下X坐标
    float vehicleY = 0;            //车辆中心在世界坐标系下Y坐标
    float vehicleZ = 0;            //车辆中心在世界坐标系下Z坐标
    float vehicleRoll = 0;         //车辆滚转角
    float vehiclePitch = 0;        //车辆俯仰角
    float vehicleYaw = 0;          //车辆横摆角
    float vehiclePitchVelocity = 0;//车辆俯仰角速度
    float vehicleRollVelocity = 0; //车辆侧倾角速度
    //车辆起始位姿
    float vehicleXRec = 0;    //车辆起点在世界坐标系下X坐标
    float vehicleYRec = 0;    //车辆起点在世界坐标系下Y坐标
    float vehicleZRec = 0;    //车辆起点在世界坐标系下Z坐标
    float vehicleRollRec = 0; //车辆起点滚转角
    float vehiclePitchRec = 0;//车辆起点俯仰角
    float vehicleYawRec = 0;  //车辆起点横摆角

    float yawVelocity = 0;                                   //车辆的横摆角速度指令
    float linearVelocity = static_cast<float>(autonomySpeed);//车辆的速度指令,赋初值

    double odomTime = 0;        //当前时间
    double slowInitTime = 0;    //减速开始时间
    double stopInitTime = false;//停车的开始时间
    int pathPointID = 0;

    int pathSize = 0;//路径点的个数

    float dirDiff = 0;//在起点坐标系下当前位置角度与到下一点连线的角度差

    float endDis = 0;//到局部路径终点的距离
    float dis = 0;   //到下一个有效点的距离
    float dis2 = 0;  //到跟踪点的距离

    bool navFwd = true;   //当前为正向行驶
    double switchTime = 0;//记录上次切换行驶方向的时间

    nav_msgs::Path path;//用于存储从localPlanner复制的路径数据


    int pubSkipCount = 0;//记录跳过次数


public://变量
    ros::Publisher pubSpeed;
    bool pathInit = false;
    const int rosRate = 100;


private://函数
    PathFollower();
    //将角度归一化到[-PI,PI]
    void normalizeAngle(float &degree);
    //将速度归一化到[-MAX,MAX]
    void normalizeVelocity(float &velocity, double max);

    //获取里程计消息回调函数，更新机器人位姿
    void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn);

    //pathHandler 函数是一个 ROS 回调函数，用于处理localPlanner传入的路径数据
    void pathHandler(const nav_msgs::Path::ConstPtr &pathIn);

    //用于处理来自速度主题的消息。函数的目的是在自主模式下，根据接收到的速度数据来调整车辆的速度
    void speedHandler(const std_msgs::Float32::ConstPtr &speed);

    //根据接收到的停止信号来更新车辆的安全停止状态
    void stopHandler(const std_msgs::Int8::ConstPtr &stop);

    //获取目标点坐标
    void goalHandler(const geometry_msgs::PointStamped::ConstPtr &goal);


public:                                //函数
    void receiveData();                //接收数据
    void printData() const;            //打印接收到的数据
    static PathFollower *getInstance();//通过调用getInstance() 在类外获得实例
    static void deleteInstance();      //删除单例
    PathFollower(const PathFollower &s) = delete;
    PathFollower &operator=(const PathFollower &s) = delete;

    //计算到下一个有效点(超过预瞄距离)的距离和角度差
    void calculateDistanceAndAngle();
    //计算目标速度和角速度
    void calculateTargetVelocityAndAngularVelocity();
    //根据最大加速度来调整速度指令的发布
    void adjustLinearVelocity();
    //停车
    void stopMoving();
    //发布指令
    void publishCommand();
};