#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

geometry_msgs::TwistStamped cmd_vel_;
const int maxSpeed = 20;
const int maxAngular = 5;

void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {


    cmd_vel_.twist.linear.x = joy->axes[1] * maxSpeed;
    cmd_vel_.twist.angular.z = joy->axes[0] * maxAngular;
    //   joyTime = ros::Time::now().toSec();//获取当前时间

    //   joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);//使用操纵杆上的轴（axes[3] 和 axes[4]）计算原始速度。这通常是通过操纵杆的倾斜程度来实现的
    //   joySpeed = joySpeedRaw;
    //   if (joySpeed > 1.0) joySpeed = 1.0;//速度比率
    //   if (joy->axes[4] == 0) joySpeed = 0;
    //   joyYaw = joy->axes[3];//转向比率
    //   if (joySpeed == 0 && noRotAtStop) joyYaw = 0;//如果速度为零,并且不允许停车时转向,转向为0
    // //不允许双向驱动情况下，如果输入手柄速度小于0，将其设置为0
    //   if (joy->axes[4] < 0 && !twoWayDrive) {
    //     joySpeed = 0;
    //     joyYaw = 0;
    //   }
    //   //通过手柄切换手动/自主模式
    //   if (joy->axes[2] > -0.1) {
    //     autonomyMode = false;
    //   } else {
    //     autonomyMode = true;
    //   }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "joy_control");
    // ros::NodeHandle nh("~");
    ros::NodeHandle nh;
    ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("joy", 5, &joystickHandler);
    ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 5);//将速度发布到cmd_vel中
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        pubSpeed.publish(cmd_vel_);
        rate.sleep();
    }
}
