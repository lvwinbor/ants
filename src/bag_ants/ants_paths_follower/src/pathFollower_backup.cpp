#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/TwistStamped.h>
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

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;      // 传感器在车体坐标系下的X值
double sensorOffsetY = 0;      // 传感器在车体坐标系下的Y值
int pubSkipNum = 1;            //机器人路径上每隔几个点发布一次
int pubSkipCount = 0;          //记录跳过次数
bool twoWayDrive = true;       // 双向驱动
double lookAheadDis = 0.5;     // 预瞄距离
double yawRateGain = 7.5;      // 转向增益
double stopYawRateGain = 7.5;  // 停车时的转向增益
double maxYawRate = 45.0;      // 最大转向速率
double maxSpeed = 1.0;         // 最大速度
double maxAccel = 1.0;         // 最大加速度
double switchTimeThre = 1.0;   // 切换方向的时间阈值
double dirDiffThre = 0.1;      // 方向差阈值，小于该值时不转向
double stopDisThre = 0.2;      // 停车距离阈值，小于该值时停车
double slowDwnDisThre = 1.0;   // 减速距离阈值，小于该值时减速
bool useInclRateToSlow = false;// 是否根据角速度来减速
double inclRateThre = 120.0;   // 关于x和y的角速度阈值,单位为度
double slowRate1 = 0.25;       // 减速率1
double slowRate2 = 0.5;        // 减速率2
double slowTime1 = 2.0;        // 减速时间1
double slowTime2 = 2.0;        // 减速时间2
bool useInclToStop = false;    // 是否根据倾斜度来停车
double inclThre = 45.0;        // roll或pitch的阈值,单位为度
double stopTime = 5.0;         // 停车时间
bool noRotAtStop = false;      // 停车时是否不旋转
bool noRotAtGoal = true;       // 到达目标点时是否不旋转
bool autonomyMode = false;     // 是否处于自主模式
double autonomySpeed = 1.0;    // 自主模式下的速度
double joyToSpeedDelay = 2.0;  // 手柄输入到自主模式速度输入的最小间隔时间
double trackDisThre = 2.0;     // 跟踪距离阈值
bool trackModeAble = true;     // 是否可以进入跟踪模式
std::string antname;

double goalX = 0;// 目标点的X坐标
double goalY = 0;// 目标点的Y坐标
//TODO:改为double
float joySpeed = 0;//速度比例
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;//安全停止信号的级别

float vehicleX = 0;    //车辆中心在世界坐标系下X坐标
float vehicleY = 0;    //车辆中心在世界坐标系下Y坐标
float vehicleZ = 0;    //车辆中心在世界坐标系下Z坐标
float vehicleRoll = 0; //车辆滚转角
float vehiclePitch = 0;//车辆俯仰角
float vehicleYaw = 0;  //车辆横摆角
//车辆起始位姿
float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;//车辆转向速度指令
float vehicleSpeed = 0;  //车辆速度指令

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;//停车的开始时间
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;   //当前为正向行驶
double switchTime = 0;//记录上次切换行驶方向的时间

nav_msgs::Path path;//用于存储从localPlanner复制的路径数据

//获取里程计消息回调函数，更新机器人位姿
void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn) {
    odomTime = odomIn->header.stamp.toSec();//从里程计消息中获取时间戳，并转换为秒

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;                                 //从里程计消息中提取四元数格式的姿态数据
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);//将四元数转换为欧拉角，分别得到滚转（roll）、俯仰（pitch）和偏航（yaw）角

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;//根据传感器中心的世界坐标求出车辆中心的世界坐标
    vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    vehicleZ = odomIn->pose.pose.position.z;

    //TODO:将下面从这个回调函数中分离出去
    //滚转角 (roll) 或俯仰角 (pitch) 是超过了一个预设阈值 (inclThre)并且允许根据倾斜度进行停车
    if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
        stopInitTime = odomIn->header.stamp.toSec();//设置停车开始时间为当前时间
    }
    //车辆的角速度（在X轴和Y轴上，弧度每秒）是否超过了预设的角速率阈值并且允许根据角速度进行减速
    if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
        slowInitTime = odomIn->header.stamp.toSec();//设置减速时间为当前时间
    }
}
//pathHandler 函数是一个 ROS 回调函数，用于处理localPlanner传入的路径数据
void pathHandler(const nav_msgs::Path::ConstPtr &pathIn) {
    int pathSize = pathIn->poses.size();//整条路径的路径点数量
    path.poses.resize(pathSize);        //分配容器的大小
    //复制路径数据
    for (int i = 0; i < pathSize; i++) {//TODO:改为vector容器的复制
        path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
        path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
        path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    }
    //TODO:不应该放在这里
    //记录车辆起始位姿
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    vehicleZRec = vehicleZ;
    vehicleRollRec = vehicleRoll;
    vehiclePitchRec = vehiclePitch;
    vehicleYawRec = vehicleYaw;

    pathPointID = 0;//初始化路径点索引
    pathInit = true;//路径初始化完成
}
//TODO:保留原速度，最后发布的时候统一处理
//用于处理来自速度主题的消息。函数的目的是在自主模式下，根据接收到的速度数据来调整车辆的速度
void speedHandler(const std_msgs::Float32::ConstPtr &speed) {
    double speedTime = ros::Time::now().toSec();
    //1.自主模式2.检查自上次接收操纵杆输入以来已过了足够的时间3.手柄输入速度为0
    if (autonomyMode) {
        joySpeed = speed->data / maxSpeed;//速度归一化

        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0)
            joySpeed = 1.0;
    }
}
//根据接收到的停止信号来更新车辆的安全停止状态
void stopHandler(const std_msgs::Int8::ConstPtr &stop) {
    safetyStop = stop->data;
}
//获取目标点坐标
void goalHandler(const geometry_msgs::PointStamped::ConstPtr &goal) {
    goalX = goal->point.x;
    goalY = goal->point.y;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pathFollower");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    //TODO:将参数放在构造函数里

    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
    nhPrivate.getParam("pubSkipNum", pubSkipNum);
    nhPrivate.getParam("twoWayDrive", twoWayDrive);
    nhPrivate.getParam("lookAheadDis", lookAheadDis);
    nhPrivate.getParam("yawRateGain", yawRateGain);
    nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
    nhPrivate.getParam("maxYawRate", maxYawRate);
    nhPrivate.getParam("maxSpeed", maxSpeed);
    nhPrivate.getParam("maxAccel", maxAccel);
    nhPrivate.getParam("switchTimeThre", switchTimeThre);
    nhPrivate.getParam("dirDiffThre", dirDiffThre);
    nhPrivate.getParam("stopDisThre", stopDisThre);
    nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
    nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
    nhPrivate.getParam("inclRateThre", inclRateThre);
    nhPrivate.getParam("slowRate1", slowRate1);
    nhPrivate.getParam("slowRate2", slowRate2);
    nhPrivate.getParam("slowTime1", slowTime1);
    nhPrivate.getParam("slowTime2", slowTime2);
    nhPrivate.getParam("useInclToStop", useInclToStop);
    nhPrivate.getParam("inclThre", inclThre);
    nhPrivate.getParam("stopTime", stopTime);
    nhPrivate.getParam("noRotAtStop", noRotAtStop);
    nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
    nhPrivate.getParam("autonomyMode", autonomyMode);
    nhPrivate.getParam("autonomySpeed", autonomySpeed);
    nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
    nhPrivate.getParam("NameSpace", antname);
    nhPrivate.getParam("trackDisThre", trackDisThre);
    nhPrivate.getParam("trackModeAble", trackModeAble);
    //TODO:将数据处理放在构造函数里

    ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped>("way_point", 5, goalHandler);

    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("state_estimation", 5, odomHandler);

    ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("path", 5, pathHandler);
    //TODO:数据类型统一
    ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("speed", 5, speedHandler);

    ros::Subscriber subStop = nh.subscribe<std_msgs::Int8>("stop", 5, stopHandler);

    ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 5);//将速度发布到cmd_vel中


    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = antname + "/vehicle";

    //TODO:删除,改为初始化
    if (autonomyMode) {
        joySpeed = autonomySpeed / maxSpeed;//将自主模式下的速度归一化

        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0)
            joySpeed = 1.0;
    }

    ros::Rate rate(100);
    bool status = ros::ok();//TODO:删除
    double last_pub_t_ = ros::Time::now().toSec();
    while (status) {
        ros::spinOnce();

        if (pathInit) {//是否经过pathHandler进行路径初始化
            //车辆在起点坐标系中的位置
            float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
            float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) + cos(vehicleYawRec) * (vehicleY - vehicleYRec);
            //算局部路径终点与车辆当前位置的X轴和Y轴差
            int pathSize = path.poses.size();
            float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
            float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
            float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);
            // if(antname == "ant02")
            // {std::cout << "------------1endx" << path.poses[pathSize - 1].pose.position.x - vehicleXRel << std::endl;
            // std::cout << "------------1endy"  << path.poses[pathSize - 1].pose.position.y - vehicleYRel << std::endl;
            // std::cout << "------------1endDis"  << endDis << std::endl;}
            //TODO:添加disX2和disY2的声明
            float disX, disY, dis, dis2;//dis2为到跟踪点的距离
            //遍历路径点直到当前位置到路径点的距离大于预设的“预瞄距离”
            while (pathPointID < pathSize - 1) {
                disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
                disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
                dis = sqrt(disX * disX + disY * disY);

                //   if(antname == "ant02")
                // {std::cout << "------------0dis"  << dis << std::endl;
                // std::cout << "------------0disx"  << disX << std::endl;
                // std::cout << "------------0disy"  << disY << std::endl;
                // std::cout << "------------0pathPointID"  << pathPointID << std::endl;}

                if (dis < lookAheadDis) {
                    pathPointID++;
                } else {
                    break;
                }
            }


            //感觉和上面重复了
            disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
            disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
            dis = sqrt(disX * disX + disY * disY);

            // float disX2 = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
            // float disY2 = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
            // dis2 = sqrt(disX2 * disX2 + disY2 * disY2);


            //计算到目标点的距离
            float disX2 = goalX - vehicleX;
            float disY2 = goalY - vehicleY;
            dis2 = sqrt(disX2 * disX2 + disY2 * disY2);

            //计算当前位置到下一个点的角度
            float pathDir = atan2(disY, disX);

            //vehicleYaw为当前位置在全局坐标系下的角度
            //vehicleYawRec为起点位置在全局坐标系下的角度
            //pathDir取值范围为（-π, π]
            //dirDiff为在起点坐标系下当前位置角度与到下一点连线的角度差
            float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
            // if(antname == "ant01")
            // {
            //   // 每隔0.3s输出一次
            // if (ros::Time::now().toSec() - last_pub_t_ > 0.5)
            // {
            // ROS_ERROR("------------1ID %d", pathPointID);
            // std::cout << "------------1ID" << pathPointID << std::endl;
            // std::cout << "------------1dirDiff" << antname << dirDiff << std::endl;
            // std::cout << "------------1vehicleYaw"  << vehicleYaw << std::endl;
            // std::cout << "------------1vehicleYawRec"  << vehicleYawRec << std::endl;
            // std::cout << "------------1pathDir"  << pathDir << std::endl;

            // std::cout << "------------1vehicleXRec"  << vehicleXRec << std::endl;
            // std::cout << "------------1vehicleYRec"  << vehicleYRec << std::endl;
            // std::cout << "------------1vehicleX"  << vehicleX << std::endl;
            // std::cout << "------------1vehicleY"  << vehicleY << std::endl;
            // last_pub_t_ = ros::Time::now().toSec();
            // }

            // }
            //TODO:可以改成循环
            //dirDiff角度转换到 -π 到 π 的范围内
            if (dirDiff > PI) dirDiff -= 2 * PI;
            else if (dirDiff < -PI)
                dirDiff += 2 * PI;
            if (dirDiff > PI) dirDiff -= 2 * PI;
            else if (dirDiff < -PI)
                dirDiff += 2 * PI;
            // if(antname == "ant02")
            // std::cout << "------------2" << antname << dirDiff << std::endl;

            //如果可以双向行使
            if (twoWayDrive) {
                double time = ros::Time::now().toSec();
                //如果方向差 (dirDiff) 的绝对值大于90度,且当前是正向行驶(navFwd=true)，并且自上次改变方向以来已经超过了一定的时间阈值 (switchTimeThre)，则切换到反向行驶
                if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
                    navFwd = false;
                    switchTime = time;
                } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
                    navFwd = true;
                    switchTime = time;
                }
            }


            //计算出车辆的目标速度（最大速度*比例）
            float joySpeed2 = maxSpeed * joySpeed;
            if (!navFwd) {                          //如果反向行使
                dirDiff += PI;                      //将角度反180度，计算反向行使角度
                if (dirDiff > PI) dirDiff -= 2 * PI;//如果调整后的 dirDiff 超出了 -π 到 π 的标准化范围，再次进行标准化
                joySpeed2 *= -1;                    //计算出的速度乘以 -1，以反映反向行驶的实际速度
            }
            // if(antname == "ant02")
            // std::cout << "------------3" << antname << dirDiff << std::endl;

            //TODO:计算转向速率
            if (trackModeAble == false) {//如果不是跟踪模式
                //车辆的速度是否低于一个特定阈值2.0 * maxAccel / 100.0（认为其停止运动）， stopYawRateGain 是在停车时使用的转向增益，dirDiff 是车辆与目标方向差,比例控制
                if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
                else
                    vehicleYawRate = -yawRateGain * dirDiff;//在正常速度下的转向,比例控制
            } else {                                        //如果是跟踪模式
                bool track = dis2 < trackDisThre;           //判断目标点是否小于跟踪距离阈值
                //如果速度低于阈值或者距离跟踪目标点过近,使用停车转向增益
                if ((fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) || track) vehicleYawRate = -stopYawRateGain * dirDiff;
                else
                    vehicleYawRate = -yawRateGain * dirDiff;
            }

            //TODO:最后发布时统一处理
            //限制转向速率范围
            if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
            else if (vehicleYawRate < -maxYawRate * PI / 180.0)
                vehicleYawRate = -maxYawRate * PI / 180.0;
            //TODO:删除手动操作
            //车辆速度为0并且手动操作
            if (joySpeed2 == 0 && !autonomyMode) {
                vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;           //根据转向比率计算转向
            } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {//检查是否路径点非常少（可能意味着没有路径或路径结束了）或者车辆到达下一个点的距离小于停车阈值（dis < stopDisThre），且在达到目标时不应该旋转
                vehicleYawRate = 0;                                          //转向为0
            }
            //TODO:计算速度
            if (pathSize <= 1) {//路径点少则说明到达终点速度设为0
                joySpeed2 = 0;
            } else if (endDis / slowDwnDisThre < joySpeed) {//到达终点的距离除以减速阈值小于速度比例
                joySpeed2 *= endDis / slowDwnDisThre;       //随着车辆接近终点，endDis减小，joySpeed2会逐渐减速
            }

            float joySpeed3 = joySpeed2;//最终速度

            // 考虑突发情况,joySpeed3改变
            //当前时间 (odomTime) 是否在某个初始减速时间点 (slowInitTime) 加上一个时间间隔 (slowTime1) 之内
            //在某个突发事件发生后的 slowTime1 时间内，将 joySpeed3（车辆速度）乘以一个减速率 slowRate1，从而减慢车辆
            if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
            //离slowInitTime时间较长时，将速度再次降低（*slowRate2）
            else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0)
                joySpeed3 *= slowRate2;

            //与下一点的方向偏差不大并且到下一点的距离大于停车阈值(车辆正朝着正确的方向前进，并且距离目标点有足够的空间)
            if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
                if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;//如果车辆速度小于（joySpeed3），车辆加速
                else if (vehicleSpeed > joySpeed3)
                    vehicleSpeed -= maxAccel / 100.0;//如果车辆当前速度高于（joySpeed3），减少车辆速度
            } else {                                 //如果车辆未朝着正确方向或太接近目标点，将车辆速度慢慢调整为0
                if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
                else if (vehicleSpeed < 0)
                    vehicleSpeed += maxAccel / 100.0;
            }

            //如果当前时间odomTime在（停车开始stopInitTime+完全停止stopTime）时间内，将车辆速度和角速度都设置为0，进行停车
            if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
                vehicleSpeed = 0;
                vehicleYawRate = 0;
            }

            //接收到不同级别的安全停止信号时，确保车辆能够适当地响应
            if (safetyStop >= 1) vehicleSpeed = 0;
            if (safetyStop >= 2) vehicleYawRate = 0;

            pubSkipCount--;                                          //减少发布跳过计数
            if (pubSkipCount < 0) {                                  //如果 pubSkipCount 小于0，表示已经达到了发布新消息的时间
                cmd_vel.header.stamp = ros::Time().fromSec(odomTime);//将 cmd_vel 消息的时间戳设置为当前里程计时间 (odomTime)
                //TODO:删除
                if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;//如果车辆的速度非常小（小于或等于最大加速度的百分之一），则将线速度设置为0，表示车辆停止
                else
                    cmd_vel.twist.linear.x = vehicleSpeed;//否则将目标速度发送
                cmd_vel.twist.angular.z = vehicleYawRate; //将目标转向发布

                // 到达跟踪距离，速度直接为0


                // 打印输出速度和距离
                if (antname == "ant0111111") {
                    std::cout << "------------cmd_vel.twist.linear.x:" << cmd_vel.twist.linear.x << std::endl;
                    std::cout << "------------dis2:" << dis2 << std::endl;
                    std::cout << "------------goalX:" << goalX << std::endl;
                    std::cout << "------------goalY:" << goalY << std::endl;
                    std::cout << "------------vehicleXRel:" << vehicleXRel << std::endl;
                    std::cout << "------------vehicleYRel:" << vehicleYRel << std::endl;
                }


                if (dis2 < trackDisThre && trackModeAble) cmd_vel.twist.linear.x = 0;

                pubSpeed.publish(cmd_vel);

                pubSkipCount = pubSkipNum;//发布新消息后重置跳点次数
            }
            // std::cout << "------------" << antname << dirDiff << std::endl;
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
