#include "pathFollower.h"
#include "common_private_msgs/controlMessage.h"
PathFollower *PathFollower::getInstance() {
    if (_pathFollower == nullptr) {//先判断是否为空，如果为空则进入，不为空说明已经存在实例，直接返回
        i_mutex.lock();
        if (_pathFollower == nullptr) {//再判断一次，确保不会因为加锁期间多个线程同时进入
            _pathFollower = new PathFollower();
        }
        i_mutex.unlock();
    }
    return _pathFollower;//返回这个实例的地址
}
void PathFollower::deleteInstance() {
    if (_pathFollower != nullptr) {
        i_mutex.lock();
        if (_pathFollower != nullptr) {
            delete _pathFollower;
            _pathFollower = nullptr;
        }
        i_mutex.unlock();
    }
}


PathFollower::PathFollower() : nhPrivate("~"),
                               sensorOffsetX(nhPrivate.param("sensorOffsetX", 0)),
                               sensorOffsetY(nhPrivate.param("sensorOffsetY", 0)),
                               pubSkipNum(nhPrivate.param("pubSkipNum", 1)),
                               twoWayDrive(nhPrivate.param("twoWayDrive", true)),
                               lookAheadDis(nhPrivate.param("lookAheadDis", 0.5)),
                               yawRateGain(nhPrivate.param("yawRateGain", 7.5)),
                               stopYawRateGain(nhPrivate.param("stopYawRateGain", 7.5)),
                               maxYawRate(nhPrivate.param("maxYawRate", 45.0)),
                               maxSpeed(nhPrivate.param("maxSpeed", 1.0)),
                               maxAccel(nhPrivate.param("maxAccel", 1.0)),
                               switchTimeThre(nhPrivate.param("switchTimeThre", 1.0)),
                               dirDiffThre(nhPrivate.param("dirDiffThre", 0.1)),
                               stopDisThre(nhPrivate.param("stopDisThre", 0.2)),
                               slowDwnDisThre(nhPrivate.param("slowDwnDisThre", 1.0)),
                               useInclRateToSlow(nhPrivate.param("useInclRateToSlow", false)),
                               inclRateThre(nhPrivate.param("inclRateThre", 120.0)),
                               slowRate1(nhPrivate.param("slowRate1", 0.25)),
                               slowRate2(nhPrivate.param("slowRate2", 0.5)),
                               slowTime1(nhPrivate.param("slowTime1", 2.0)),
                               slowTime2(nhPrivate.param("slowTime2", 2.0)),
                               useInclToStop(nhPrivate.param("useInclToStop", false)),
                               inclThre(nhPrivate.param("inclThre", 45.0)),
                               stopTime(nhPrivate.param("stopTime", 5.0)),
                               noRotAtStop(nhPrivate.param("noRotAtStop", false)),
                               noRotAtGoal(nhPrivate.param("noRotAtGoal", true)),
                               autonomySpeed(nhPrivate.param("autonomySpeed", 1.0)),
                               joyToSpeedDelay(nhPrivate.param("joyToSpeedDelay", 2.0)),
                               antname(nhPrivate.param("NameSpace", std::string("ant01"))),
                               trackDisThre(nhPrivate.param("trackDisThre", 2.0)),
                               trackModeAble(nhPrivate.param("trackModeAble", true)) {
    subGoal = nh.subscribe<geometry_msgs::PointStamped>("way_point", 5, &PathFollower::goalHandler, this);

    subOdom = nh.subscribe<nav_msgs::Odometry>("state_estimation", 5, &PathFollower::odomHandler, this);

    subPath = nh.subscribe<nav_msgs::Path>("path", 5, &PathFollower::pathHandler, this);

    subSpeed = nh.subscribe<std_msgs::Float32>("speed", 5, &PathFollower::speedHandler, this);

    subStop = nh.subscribe<std_msgs::Int8>("stop", 5, &PathFollower::stopHandler, this);

    pubSpeed = nh.advertise<common_private_msgs::controlMessage>("autonomyCommand", 5);//将速度发布到cmd_vel中
}


void PathFollower::normalizeAngle(float &degree) {
    while (degree > PI || degree < -PI) {
        if (degree > 0) {
            degree -= 2 * PI;
        } else {
            degree += 2 * PI;
        }
    }
}
void PathFollower::normalizeVelocity(float &velocity, double max) {
    if (velocity > max) {
        velocity = static_cast<float>(max);
    } else if (velocity < -max) {
        velocity = static_cast<float>(-max);
    }
}
void PathFollower::calculateDistanceAndAngle() {
    //车辆在起点坐标系中的位置
    float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
    float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) + cos(vehicleYawRec) * (vehicleY - vehicleYRec);
    //算局部路径终点与车辆当前位置的X轴和Y轴差
    pathSize = static_cast<int>(path.poses.size());
    float endDisX = static_cast<float>(path.poses[pathSize - 1].pose.position.x - vehicleXRel);
    float endDisY = static_cast<float>(path.poses[pathSize - 1].pose.position.y - vehicleYRel);
    endDis = sqrt(endDisX * endDisX + endDisY * endDisY);
    float disX, disY;//
    //遍历路径点直到当前位置到路径点的距离大于预设的“预瞄距离”
    while (pathPointID < pathSize - 1) {
        disX = static_cast<float>(path.poses[pathPointID].pose.position.x - vehicleXRel);
        disY = static_cast<float>(path.poses[pathPointID].pose.position.y - vehicleYRel);
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
            pathPointID++;
        } else {
            break;
        }
    }


    //计算到目标点的距离
    float disX2 = static_cast<float>(goalX - vehicleX);
    float disY2 = static_cast<float>(goalY - vehicleY);
    dis2 = sqrt(disX2 * disX2 + disY2 * disY2);

    //计算当前位置到下一个点的角度
    float pathDir = atan2(disY, disX);

    //vehicleYaw为当前位置在全局坐标系下的角度
    //vehicleYawRec为起点位置在全局坐标系下的角度
    //pathDir取值范围为（-π, π]
    dirDiff = vehicleYaw - vehicleYawRec - pathDir;

    normalizeAngle(dirDiff);
}
void PathFollower::calculateTargetVelocityAndAngularVelocity() {
    if (twoWayDrive) {//如果允许双向行驶
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

    if (!navFwd) {    //如果反向行使
        dirDiff += PI;//将角度反180度，计算反向行使角度
        normalizeAngle(dirDiff);
        targetVelocity *= -1;//计算出的速度乘以 -1，以反映反向行驶的实际速度
    }
    //计算转向速率
    //车辆的速度是否低于一个特定阈值2.0 * maxAccel / 100.0（认为其停止运动）或者跟踪模式下距离跟踪目标点过近，使用停车转向增益
    if ((fabs(linearVelocity) < 2.0 * maxAccel / rosRate) || (trackModeAble && (dis2 < trackDisThre))) {
        yawVelocity = static_cast<float>(-stopYawRateGain * dirDiff);
    } else {//在正常速度下的转向,比例控制
        yawVelocity = static_cast<float>(-yawRateGain * dirDiff);
    }

    //限制转向速率范围
    normalizeVelocity(yawVelocity, maxYawRate * PI / 180.0);
    //检查是否路径点非常少（可能意味着没有路径或路径结束了）或者车辆到达下一个点的距离小于停车阈值（dis < stopDisThre），且在达到目标时不应该旋转
    if (pathSize <= 1 || ((dis < stopDisThre) && noRotAtGoal)) {
        yawVelocity = 0;
    }
    //计算速度
    if (pathSize <= 1) {//路径点少则说明到达终点速度设为0
        targetVelocity = 0;
    } else if ((endDis / slowDwnDisThre) < (targetVelocity / maxSpeed)) {//到达终点的距离除以减速阈值小于速度比例
        targetVelocity *= static_cast<float>(endDis / slowDwnDisThre);   //随着车辆接近终点，endDis减小，joySpeed2会逐渐减速
    }


    //车辆的角速度（在X轴和Y轴上，弧度每秒）是否超过了预设的角速率阈值并且允许根据角速度进行减速
    if ((fabs(vehicleRollVelocity) > inclRateThre * PI / 180.0 || fabs(vehiclePitchVelocity) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
        slowInitTime = odomTime;//设置减速时间为当前时间
    }
    //当前时间 (odomTime) 是否在某个初始减速时间点 (slowInitTime) 加上一个时间间隔 (slowTime1) 之内
    //在特殊事件发生后的 slowTime1 时间内，将 joySpeed3（车辆速度）乘以一个减速率 slowRate1，从而减慢车辆
    if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) {
        targetVelocity *= static_cast<float>(slowRate1);
    }
    //离slowInitTime时间较长时，将速度再次降低（*slowRate2）
    else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) {
        targetVelocity *= static_cast<float>(slowRate2);
    }
}
void PathFollower::adjustLinearVelocity() {
    //与下一点的方向偏差不大并且到下一点的距离大于停车阈值(车辆正朝着正确的方向前进，并且距离目标点有足够的空间)
    if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (linearVelocity < targetVelocity) {
            linearVelocity += static_cast<float>(maxAccel / rosRate);
        }//如果车辆速度小于（joySpeed3），车辆加速
        else if (linearVelocity > targetVelocity) {
            linearVelocity -= static_cast<float>(maxAccel / rosRate);
        }//如果车辆当前速度高于（joySpeed3），减少车辆速度
    } else {//如果车辆未朝着正确方向或太接近目标点，将车辆速度慢慢调整为0
        if (linearVelocity > 0) {
            linearVelocity -= static_cast<float>(maxAccel / rosRate);
        } else if (linearVelocity < 0) {
            linearVelocity += static_cast<float>(maxAccel / rosRate);
        }
    }
}
void PathFollower::stopMoving() {
    //滚转角 (roll) 或俯仰角 (pitch) 是超过了一个预设阈值 (inclThre)并且允许根据倾斜度进行停车
    if ((fabs(vehicleRoll) > inclThre * PI / 180.0 || fabs(vehiclePitch) > inclThre * PI / 180.0) && useInclToStop) {
        stopInitTime = odomTime;//设置停车开始时间为当前时间
    }
    //如果当前时间odomTime在（停车开始stopInitTime+完全停止stopTime）时间内，将车辆速度和角速度都设置为0，进行停车
    if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        linearVelocity = 0;
        yawVelocity = 0;
    }
    //接收到不同级别的安全停止信号时，确保车辆能够适当地响应
    if (safetyStop >= 1) {
        linearVelocity = 0;
    }
    if (safetyStop >= 2) {
        yawVelocity = 0;
    }
}
void PathFollower::publishCommand() {
    pubSkipCount--;                                          //减少发布跳过计数
    if (pubSkipCount < 0) {                                  //如果 pubSkipCount 小于0，表示已经达到了发布新消息的时间
        command.header.stamp = ros::Time().fromSec(odomTime);//将 command 消息的时间戳设置为当前里程计时间 (odomTime)
        //如果车辆的速度非常小（小于或等于最大加速度的百分之一），则将线速度设置为0，表示车辆停止
        if (fabs(linearVelocity) <= maxAccel / rosRate) {
            command.linearVelocity = 0;
        }//否则将目标速度发送
        else {
            command.linearVelocity = linearVelocity;
        }
        command.yawVelocity = yawVelocity;//将目标转向发布

        // 到达跟踪距离，速度直接为0
        if (dis2 < trackDisThre && trackModeAble) {
            command.linearVelocity = 0;
        }
        command.header.frame_id = antname + "/vehicle";

        pubSpeed.publish(command);

        pubSkipCount = pubSkipNum;//发布新消息后重置跳点次数
    }
}
void PathFollower::odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn) {
    odomTime = odomIn->header.stamp.toSec();//从里程计消息中获取时间戳，并转换为秒

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;                                 //从里程计消息中提取四元数格式的姿态数据
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);//将四元数转换为欧拉角，分别得到滚转（roll）、俯仰（pitch）和偏航（yaw）角

    vehicleRoll = static_cast<float>(roll);
    vehiclePitch = static_cast<float>(pitch);
    vehicleYaw = static_cast<float>(yaw);
    vehicleX = static_cast<float>(odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY);//根据传感器中心的世界坐标求出车辆中心的世界坐标
    vehicleY = static_cast<float>(odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY);
    vehicleZ = static_cast<float>(odomIn->pose.pose.position.z);

    vehicleRollVelocity = static_cast<float>(odomIn->twist.twist.angular.x);
    vehiclePitchVelocity = static_cast<float>(odomIn->twist.twist.angular.y);
}
void PathFollower::pathHandler(const nav_msgs::Path::ConstPtr &pathIn) {
    int pathSize = static_cast<int>(pathIn->poses.size());//整条路径的路径点数量
    //复制路径数据
    path.poses = pathIn->poses;

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
void PathFollower::speedHandler(const std_msgs::Float32::ConstPtr &unsignedSpeed) {
    targetVelocity = unsignedSpeed->data;
    normalizeVelocity(targetVelocity, maxSpeed);
}
void PathFollower::stopHandler(const std_msgs::Int8::ConstPtr &stop) {
    safetyStop = stop->data;
}
void PathFollower::goalHandler(const geometry_msgs::PointStamped::ConstPtr &goal) {
    goalX = goal->point.x;
    goalY = goal->point.y;
}

PathFollower *PathFollower::_pathFollower = nullptr;
std::mutex PathFollower::i_mutex;
int main(int argc, char **argv) {
    ros::init(argc, argv, "pathFollower");
    PathFollower *pathFollower = PathFollower::getInstance();

    ros::Rate rate(pathFollower->rosRate);
    while (ros::ok()) {
        ros::spinOnce();
        if (pathFollower->pathInit) {
            pathFollower->calculateDistanceAndAngle();
            pathFollower->calculateTargetVelocityAndAngularVelocity();
            pathFollower->adjustLinearVelocity();
            pathFollower->stopMoving();
            pathFollower->publishCommand();
        }
        rate.sleep();
    }
    PathFollower::deleteInstance();
}
