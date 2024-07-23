#include <common_private_msgs/controlMessage.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


common_private_msgs::controlMessage joyCommand;
const int maxSpeed = 1;
const int maxAngular = 2;
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
    //axes数据为float
    //buttons数据为int

    joyCommand.linearVelocity = joy->axes[1] * maxSpeed;
    joyCommand.yawVelocity = joy->axes[0] * maxAngular;
    static bool joyMode = false;
    static bool lastButtonState = false;//按键上一个状态
    bool currentButtonState = static_cast<bool>(joy->buttons[11]);
    if (!currentButtonState && lastButtonState) {//当按键从1变为0，才进行模式切换
        joyMode = !joyMode;
    }
    joyCommand.joyControlMode = joyMode;
    lastButtonState = currentButtonState;//更新上一个状态


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
    ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("joy", 1, &joystickHandler);
    ros::Publisher pubJoystick = nh.advertise<common_private_msgs::controlMessage>("joyCommand", 1);//将速度发布到cmd_vel中
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        pubJoystick.publish(joyCommand);
        rate.sleep();
    }
}
